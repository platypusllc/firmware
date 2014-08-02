package edu.cmu.ri.airboat.server;

import android.content.Context;
import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import android.util.JsonWriter;
import android.util.Log;

import com.google.code.microlog4android.LoggerFactory;

import org.json.JSONException;
import org.json.JSONObject;

import java.io.IOException;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.Arrays;
import java.util.Iterator;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.atomic.AtomicBoolean;

import edu.cmu.ri.crw.AbstractVehicleServer;
import edu.cmu.ri.crw.VehicleController;
import edu.cmu.ri.crw.VehicleFilter;
import edu.cmu.ri.crw.VehicleServer;
import edu.cmu.ri.crw.data.SensorData;
import edu.cmu.ri.crw.data.Twist;
import edu.cmu.ri.crw.data.Utm;
import edu.cmu.ri.crw.data.UtmPose;
import robotutils.Pose3D;

/**
 * Contains the actual implementation of vehicle functionality, accessible as a
 * singleton that is updated and maintained by a background service.
 * 
 * @author pkv
 * @author kss
 * 
 */
public class AirboatImpl extends AbstractVehicleServer {

	private static final com.google.code.microlog4android.Logger logger = LoggerFactory
			.getLogger();

	private static final String logTag = AirboatImpl.class.getName();
	public static final int UPDATE_INTERVAL_MS = 100;
	public static final int NUM_SENSORS = 4;
	public static final VehicleController DEFAULT_CONTROLLER = AirboatController.STOP.controller;

    protected final SharedPreferences _prefs;

    protected final SensorType[] _sensorTypes = new SensorType[NUM_SENSORS];
	protected UtmPose[] _waypoints = new UtmPose[0];

	protected final Object _captureLock = new Object();
	protected TimerTask _captureTask = null;

	protected final Object _navigationLock = new Object();
	protected TimerTask _navigationTask = null;

	private final Timer _updateTimer = new Timer();
	private final Timer _navigationTimer = new Timer();
	private final Timer _captureTimer = new Timer();

    // TODO: Remove this variable, it is totally arbitrary
    private double winch_depth_ = Double.NaN;

    public static enum VehicleType {
        DIFFERENTIAL_THRUST,
        VECTORED_THRUST,
        UNKNOWN
    }

    /**
	 * Defines the PID gains that will be returned if there is an error.
	 */
	public static final double[] NAN_GAINS = new double[] { Double.NaN,
			Double.NaN, Double.NaN };

	// Status information
	final AtomicBoolean _isConnected = new AtomicBoolean(true);
	final AtomicBoolean _isAutonomous = new AtomicBoolean(false);

	// Internal data structures for Amarino callbacks
	final Context _context;
	final PrintWriter _usbWriter;
	public static final double[] DEFAULT_TWIST = {0, 0, 0, 0, 0, 0};

	/**
	 * Inertial state vector, currently containing a 6D pose estimate:
	 * [x,y,z,roll,pitch,yaw]
	 */
	UtmPose _utmPose = new UtmPose(new Pose3D(476608.34, 4671214.40, 172.35, 0, 0, 0), new Utm(17, true));

	/**
	 * Filter used internally to update the current pose estimate
	 */
	VehicleFilter filter = new SimpleFilter();

	/**
	 * Inertial velocity vector, containing a 6D angular velocity estimate: [rx,
	 * ry, rz, rPhi, rPsi, rOmega]
	 */
	Twist _velocities = new Twist(DEFAULT_TWIST);

    /**
	 * Raw gyroscopic readings from the phone gyro. 
	 */
	final double[] _gyroPhone = new double[3];

    /**
	 * Hard-coded PID gains and thrust limits per vehicle type.
	 */
	double[] r_PID = {.2, 0, .3}; // Kp, Ki, Kd
	double [] t_PID = {.5, .5, .5};
    public static final double SAFE_DIFFERENTIAL_THRUST = 0.4;
    public static final double SAFE_VECTORED_THRUST = 0.6;

    /**
     * Simple clipping function that restricts a value to a given range.
     * @param input value that needs to be clipped
     * @param min minimum allowable value
     * @param max maximum allowable value
     * @return value after it has been clipped between min and max.
     */
    public static double clip(double input, double min, double max) {
        return Math.min(Math.max(input, min), max);
    }

    /**
     * Simple linear scaling function that maps a value from a given input range to a desired output range.
     *
     * This does *not* clip out of range values.  To invert values, swap min and max.
     *
     * @param input value that needs to be scaled
     * @param input_min lower bound of original mapping
     * @param input_max upper bound of original mapping
     * @param output_min lower bound of desired mapping
     * @param output_max upper bound of desired mapping.
     * @return the input value mapped into the output range.
     */
    public static double map(double input,
                              double input_min, double input_max,
                              double output_min, double output_max) {
        return (input - input_min) / (input_max - input_min)
                * (output_max - output_min) + output_min;
    }

	/**
	 * Creates a new instance of the vehicle implementation. This function
	 * should only be used internally when the corresponding vehicle service is
	 * started and stopped.
	 * 
	 * @param context
	 *            the application context to use
	 * @param usbWriter
	 *            the USB device for the vehicle controller
	 */

	protected AirboatImpl(Context context, PrintWriter usbWriter) {
		_context = context;
		_usbWriter = usbWriter;

        // Connect to the Shared Preferences for this process.
        _prefs = PreferenceManager.getDefaultSharedPreferences(_context);

        // Start a regular update function
		_updateTimer.scheduleAtFixedRate(_updateTask, 0, UPDATE_INTERVAL_MS);
	}

	/**
	 * Internal update function called at regular intervals to process command
	 * and control events.
	 */
	private TimerTask _updateTask = new TimerTask() {

		@Override
		public void run() {
			/*
			 * // Compute the number of milliseconds since last update // (or 0
			 * if this is the first update) long currentUpdateMs =
			 * SystemClock.elapsedRealtime(); long elapsedMs = (_lastUpdateMs >
			 * 0) ? currentUpdateMs - _lastUpdateMs : 0; _lastUpdateMs =
			 * currentUpdateMs;
			 */
			// Do an intelligent state prediction update here
			_utmPose = filter.pose(System.currentTimeMillis());
			logger.info("POSE: " + _utmPose);
			sendState(_utmPose.clone());

            // Determine current vehicle type from ordinal value.
            int vehicle_type_index = _prefs.getInt(AirboatActivity.KEY_VEHICLE_TYPE, VehicleType.UNKNOWN.ordinal());
            VehicleType vehicle_type = VehicleType.values()[vehicle_type_index];

            // Send vehicle command by converting raw command to appropriate vehicle model.
            JSONObject command = new JSONObject();
            switch (vehicle_type) {
                case DIFFERENTIAL_THRUST:
                    // Construct objects to hold velocities
                    JSONObject velocity0 = new JSONObject();
                    JSONObject velocity1 = new JSONObject();

                    // Send velocities as a JSON command
                    try {
                        // Until ESCs are able to reverse, set the lower limit to 0.0
                        double constrainedV0 = clip(_velocities.dx() - _velocities.drz(), -1.0, 1.0);
                        double constrainedV1 = clip(_velocities.dx() + _velocities.drz(), -1.0, 1.0);

                        // Until ESC reboot is fixed, set the upper limit to SAFE_THRUST
                        constrainedV0 = map(constrainedV0,
                                -1.0, 1.0, // Original range.
                                -AirboatImpl.SAFE_DIFFERENTIAL_THRUST, AirboatImpl.SAFE_DIFFERENTIAL_THRUST); // New range.
                        constrainedV1 = map(constrainedV1,
                                -1.0, 1.0, // Original range.
                                -AirboatImpl.SAFE_DIFFERENTIAL_THRUST, AirboatImpl.SAFE_DIFFERENTIAL_THRUST); // New range.

                        velocity0.put("v", (float) constrainedV0);
                        velocity1.put("v", (float) constrainedV1);

                        command.put("m0", velocity0);
                        command.put("m1", velocity1);

                        _usbWriter.println(command.toString());
                        _usbWriter.flush();

                        logger.info("CMD: " + command.toString());
                    } catch (JSONException e) {
                        Log.w(logTag, "Failed to serialize command.", e); // TODO: remove this.
                    }
                    break;

                case VECTORED_THRUST:
                    // Construct objects to hold velocities
                    JSONObject thrust = new JSONObject();
                    JSONObject rudder = new JSONObject();

                    // Send velocities as a JSON command
                    try {
                        // Until ESCs are able to reverse, set the lower limit to 0.0
                        double constrainedV = clip(_velocities.dx(), 0.0, 1.0);

                        // Until ESC reboot is fixed, set the upper limit to SAFE_THRUST
                        constrainedV = map(constrainedV,
                                0.0, 1.0, // Original range.
                                0.0, AirboatImpl.SAFE_VECTORED_THRUST); // New range.

                        // Rudder is constrained to +/-1.0
                        double constrainedP = clip(_velocities.drz(), -1.0, 1.0);

                        thrust.put("v", (float) constrainedV);
                        rudder.put("p", (float) constrainedP);

                        command.put("m0", thrust);
                        command.put("s0", rudder);

                        _usbWriter.println(command.toString());
                        _usbWriter.flush();

                        logger.info("CMD: " + command.toString());
                    } catch (JSONException e) {
                        Log.w(logTag, "Failed to serialize command.", e); // TODO: remove this.
                    }
                    break;
                default:
                    Log.w(logTag, "Unknown vehicle type: " + vehicle_type);
            }
        }
	};

	/**
	 * @see VehicleServer#getGains(int)
	 */
	@Override
	public double[] getGains(int axis) {
		
		if (axis == 5)
			return r_PID.clone();
		else if (axis == 0)
			return t_PID.clone();
		else if (axis == 3)
            return new double[]{ winch_depth_, 0.0, 0.0 };
        else
            return NAN_GAINS;
	}

	/**
	 * @see VehicleServer#setGains(int, double[])
	 */
	@Override
	public void setGains(int axis, double[] k) {
		if (axis == 5) {
			r_PID = k.clone();
            logger.info("SETGAINS: " + axis + " " + Arrays.toString(k));
        } else if (axis == 0) {
			t_PID = k.clone();
            logger.info("SETGAINS: " + axis + " " + Arrays.toString(k));
        }
        // TODO: Get rid of this, it is a hack.
        // Special case to handle winch commands...
        else if (axis == 3) {
            JSONObject command = new JSONObject();
            JSONObject winchSettings = new JSONObject();

            logger.info("WINCH: " + Arrays.toString(k));
            // Call command to adjust winch
            synchronized (_usbWriter) {
                try{
                    //Set desired winch movement distance
                    winchSettings.put("p", (float) Math.abs(k[0]));

                    //Hardcoded velocity - get rid of this eventually
                    winchSettings.put("v", 500*Math.signum(k[0]));
                    command.put("s2", winchSettings);
                    _usbWriter.println(command.toString());
                    _usbWriter.flush();
                    logger.info("WINCH CMD: " + command.toString());
                } catch (Exception e) {
                    Log.w(logTag, "Unable to construct JSON string from winch command: " + Arrays.toString(k));
                }
            }
        }
	}
	/**
	 * Returns the current gyro readings
	 */
	public double[] getGyro()
	{
		return _gyroPhone.clone();
	}
	
	public void setPhoneGyro(float[] gyroValues)
	{
		for (int i = 0; i < gyroValues.length; i++)
			_gyroPhone[i] =  (double) gyroValues[i];
	}
	
	/**
	 * @see edu.cmu.ri.crw.VehicleServer#isConnected()
	 */
	public boolean isConnected() {
		return _isConnected.get();
	}
	/**
	 * Internal function used to set the connection status of this object
	 * (indicating whether currently in contact with vehicle controller).
	 */
	protected void setConnected(boolean isConnected) {
		_isConnected.set(isConnected);
	}

	/**
	 * Handles complete Arduino commands, once they are reassembled.
	 * 
	 * @param cmd
	 *            the list of arguments composing a command
	 */
	protected void onCommand(JSONObject cmd) {

		@SuppressWarnings("unchecked")
		Iterator<String> keyIterator = (Iterator<String>)cmd.keys();

		// Iterate through JSON fields
		while (keyIterator.hasNext()) {
			String name = keyIterator.next();
			try {
				JSONObject value = cmd.getJSONObject(name);
				
				if (name.startsWith("m")) {
					int motor = name.charAt(1) - 48;
					logger.info("MOTOR" + motor + ": " + value.getDouble("v"));
				} else if (name.startsWith("s")) {
					int sensor = name.charAt(1) - 48;
					logger.info("SENSOR" + sensor + ": " + value.toString());
					
					// Hacks to send sensor information
					if (value.has("type")) {
						String type = value.getString("type");
						if (type.equalsIgnoreCase("es2")) {
							SensorData reading = new SensorData();
							reading.channel = sensor;
                            reading.type = SensorType.TE;
							reading.data = new double[] {
									value.getDouble("t"),
									value.getDouble("c")
								};
							sendSensor(sensor, reading);
					    } else if (type.equalsIgnoreCase("hdf5")) {
                            String nmea = value.getString("nmea");
                            if (nmea.startsWith("$SDDBT")) {
                                try {
                                    double depth = Double.parseDouble(nmea.split(",")[3]);

                                    SensorData reading = new SensorData();
                                    reading.type = SensorType.DEPTH;
                                    reading.channel = sensor;
                                    reading.data = new double[] { depth };

                                    sendSensor(sensor, reading);
                                } catch(Exception e) {
                                    Log.w(logTag, "Failed to parse depth reading: " + nmea);
                                }
                            }
                        } else if (type.equalsIgnoreCase("winch")) {
                            SensorData reading = new SensorData();
                            reading.channel = sensor;
                            reading.type = SensorType.UNKNOWN;
                            reading.data = new double[] {
                                    value.getDouble("depth")
                            };
                            sendSensor(sensor, reading);

                            // TODO: Remove this hack to store winch depth
                            winch_depth_ = reading.data[0];
                        }
                    }
				} else {
					Log.w(logTag, "Received unknown param '" + cmd + "'.");
				}
			} catch (JSONException e) {
				Log.w(logTag, "Malformed JSON command '" + cmd + "'.", e);
			}
		}
	}

	// TODO: Revert capture image to take images
	// This is a hack to support the water sampler until PID is working again.
	public synchronized byte[] captureImage(int width, int height) {
		// Call command to fire sampler
		synchronized (_usbWriter) {
			_usbWriter.println("{\"s0\":{ \"sample\": true }");
			_usbWriter.flush();
		}
		Log.i(logTag, "Triggering sampler.");
		logger.info("SMP: NOW");
		return new byte[1];
	}
	
	public synchronized byte[] captureImageInternal(int width, int height) {
		byte[] bytes = AirboatCameraActivity.takePhoto(_context, width, height);
		Log.i(logTag, "Sending image [" + bytes.length + "]");
		return bytes;
	}

	public synchronized boolean saveImage() {
		AirboatCameraActivity.savePhoto(_context);
		Log.i(logTag, "Saving image.");
		return true;
	}

	@Override
	public void startCamera(final int numFrames, final double interval,
			final int width, final int height) {
		Log.i(logTag, "Starting capture: " + numFrames + "(" + width + "x"
				+ height + ") frames @ " + interval + "s");

		// Create a camera capture task
		TimerTask newCaptureTask = new TimerTask() {
			int iFrame = 0;

			@Override
			public void run() {
				synchronized (_captureLock) {
					// Take a new image and send it out
					sendImage(captureImageInternal(width, height));
					iFrame++;

					// If we exceed numFrames, we finished
					if (numFrames > 0 && iFrame >= numFrames) {
						sendCameraUpdate(CameraState.DONE);
						this.cancel();
						_captureTask = null;
					} else {
						sendCameraUpdate(CameraState.CAPTURING);
					}
				}
			}
		};

		synchronized (_captureLock) {
			// Cancel any previous capture tasks
			if (_captureTask != null)
				_captureTask.cancel();

			// Schedule this task for execution
			_captureTask = newCaptureTask;
			_captureTimer.scheduleAtFixedRate(_captureTask, 0,
					(long) (interval * 1000.0));
		}

		// Report the new imaging job in the log file
		logger.info("IMG: " + numFrames + " @ " + interval + "s, " + width
				+ " x " + height);
	}

	@Override
	public void stopCamera() {
		// Stop the thread that sends out images by terminating its
		// navigation flag and then removing the reference to the old flag.
		synchronized (_captureLock) {
			if (_captureTask != null) {
				_captureTask.cancel();
				_captureTask = null;
			}
		}
		sendCameraUpdate(CameraState.CANCELLED);
	}

	@Override
	public CameraState getCameraStatus() {
		synchronized (_captureLock) {
			if (_captureTask != null) {
				return CameraState.CAPTURING;
			} else {
				return CameraState.OFF;
			}
		}
	}

	@Override
	public SensorType getSensorType(int channel) {
		return _sensorTypes[channel];
	}

	@Override
	public void setSensorType(int channel, SensorType type) {
		_sensorTypes[channel] = type;
	}

	@Override
	public int getNumSensors() {
		return NUM_SENSORS;
	}

	@Override
	public UtmPose getPose() {
		return _utmPose;
	}
	/**
	 * Takes a 6D vehicle pose, does appropriate internal computation to change
	 * the current estimate of vehicle state to match the specified pose. Used
	 * for user- or multirobot- pose corrections.
	 * 
	 * @param pose
	 *            the corrected 6D pose of the vehicle: [x,y,z,roll,pitch,yaw]
	 */
	@Override
	public void setPose(UtmPose pose) {

		// Change the offset of this vehicle by modifying filter
		filter.reset(pose, System.currentTimeMillis());

		// Copy this pose over the existing value
		_utmPose = pose.clone();

		// Report the new pose in the log file
		logger.info("POSE: " + _utmPose);
	}

	@Override
	public void startWaypoints(final UtmPose[] waypoints,
			final String controller) {
		Log.i(logTag,
				"Starting waypoints with " + controller + ": "
						+ Arrays.toString(waypoints));
		if (controller.equalsIgnoreCase("PRIMITIVES"))
		{
			_waypoints = new UtmPose[waypoints.length];
			System.arraycopy(waypoints, 0, _waypoints, 0, _waypoints.length);
			VehicleController vc = AirboatController.valueOf(controller).controller;
			vc.update(AirboatImpl.this, (double) UPDATE_INTERVAL_MS / 1000.0);
		}
		else
		{
			// Create a waypoint navigation task
			TimerTask newNavigationTask = new TimerTask() {
				final double dt = (double) UPDATE_INTERVAL_MS / 1000.0;

				// Retrieve the appropriate controller in initializer
				VehicleController vc = DEFAULT_CONTROLLER;
				{
					try {
						vc = (controller == null) ? vc : AirboatController.valueOf(controller).controller;
					} catch (IllegalArgumentException e) {
						Log.w(logTag, "Unknown controller specified (using " + vc
								+ " instead): " + controller);
					}
				}

				@Override
				public void run() {
					synchronized (_navigationLock) {
						if (!_isAutonomous.get()) {
							// If we are not autonomous, do nothing
							sendWaypointUpdate(WaypointState.PAUSED);
						} else if (_waypoints.length == 0) {
							// If we are finished with waypoints, stop in place
							sendWaypointUpdate(WaypointState.DONE);
							setVelocity(new Twist(DEFAULT_TWIST));
							this.cancel();
							_navigationTask = null;
						} else {
							// If we are still executing waypoints, use a
							// controller to figure out how to get to waypoint
							// TODO: measure dt directly instead of approximating
							vc.update(AirboatImpl.this, dt);
							sendWaypointUpdate(WaypointState.GOING);
						}
					}
				}
			};

			synchronized (_navigationLock) {
				// Change waypoints to new set of waypoints
				_waypoints = new UtmPose[waypoints.length];
				System.arraycopy(waypoints, 0, _waypoints, 0, _waypoints.length);

				// Cancel any previous navigation tasks
				if (_navigationTask != null)
					_navigationTask.cancel();

				// Schedule this task for execution
				_navigationTask = newNavigationTask;
				_navigationTimer.scheduleAtFixedRate(_navigationTask, 0, UPDATE_INTERVAL_MS);
			}

			// Report the new waypoint in the log file
			logger.info("NAV: " + controller + " " + Arrays.toString(waypoints));
		}
	}

	@Override
	public void stopWaypoints() {
		// Stop the thread that is doing the "navigation" by terminating its
		// navigation process, clear all the waypoints, and stop the vehicle.
		synchronized (_navigationLock) {
			if (_navigationTask != null) {
				_navigationTask.cancel();
				_navigationTask = null;
				_waypoints = new UtmPose[0];
				setVelocity(new Twist(DEFAULT_TWIST));
			}
		}
		sendWaypointUpdate(WaypointState.CANCELLED);
	}

	@Override
	public UtmPose[] getWaypoints() {
		UtmPose[] wpts = new UtmPose[_waypoints.length];
		synchronized (_navigationLock) {
			System.arraycopy(_waypoints, 0, wpts, 0, wpts.length);
		}
		return wpts;
	}

	@Override
	public WaypointState getWaypointStatus() {
		synchronized (_navigationLock) {
			if (_waypoints.length > 0) {
				return _isAutonomous.get() ? WaypointState.PAUSED
						: WaypointState.GOING;
			} else {
				return WaypointState.DONE;
			}
		}
	}

	/**
	 * Returns the current estimated 6D velocity of the vehicle.
	 */
	public Twist getVelocity() {
		return _velocities.clone();
	}

	/**
	 * Sets a desired 6D velocity for the vehicle.
	 */
	public void setVelocity(Twist vel) {
		_velocities = vel.clone();
	}
	@Override
	public boolean isAutonomous() {
		return _isAutonomous.get();
	}

	@Override
	public void setAutonomous(boolean isAutonomous) {
		_isAutonomous.set(isAutonomous);

		// Set velocities to zero to allow for safer transitions
		_velocities = new Twist(DEFAULT_TWIST);
	}

	/**
	 * Performs cleanup functions in preparation for stopping the server.
	 */
	public void shutdown() {
		stopWaypoints();
		stopCamera();

		_isAutonomous.set(false);
		_isConnected.set(false);

		_updateTimer.cancel();
		_updateTimer.purge();
		
		_navigationTimer.cancel();
		_navigationTimer.purge();
		
		_captureTimer.cancel();
		_captureTimer.purge();
	}
}
