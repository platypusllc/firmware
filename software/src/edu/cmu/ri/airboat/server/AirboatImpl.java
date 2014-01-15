package edu.cmu.ri.airboat.server;

import java.io.FileDescriptor;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.atomic.AtomicBoolean;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import robotutils.Pose3D;
import android.content.Context;
import android.os.ParcelFileDescriptor;
import android.util.Log;

import com.google.code.microlog4android.LoggerFactory;

import edu.cmu.ri.crw.AbstractVehicleServer;
import edu.cmu.ri.crw.VehicleController;
import edu.cmu.ri.crw.VehicleFilter;
import edu.cmu.ri.crw.VehicleServer;
import edu.cmu.ri.crw.data.SensorData;
import edu.cmu.ri.crw.data.Twist;
import edu.cmu.ri.crw.data.Utm;
import edu.cmu.ri.crw.data.UtmPose;

/**
 * Contains the actual implementation of vehicle functionality, accessible as a
 * singleton that is updated and maintained by a background service.
 * 
 * @author pkv
 * @author kss
 * 
 */
public class AirboatImpl extends AbstractVehicleServer {

	private static final String logTag = AirboatImpl.class.getName();
	private static final com.google.code.microlog4android.Logger logger = LoggerFactory.getLogger();
	
	public static final String OBSTACLE = "avoidObstacle";
	public static final int UPDATE_INTERVAL_MS = 200;
	public static final int NUM_SENSORS = 4;
	public static final AirboatController DEFAULT_CONTROLLER = AirboatController.POINT_AND_SHOOT;

	private static final Charset US_ASCII = Charset.forName("US-ASCII");
	
	protected final SensorType[] _sensorTypes = new SensorType[NUM_SENSORS];
	protected UtmPose[] _waypoints = new UtmPose[0];

	protected final Object _captureLock = new Object();
	protected TimerTask _captureTask = null;

	protected final Object _navigationLock = new Object();
	protected TimerTask _navigationTask = null;

	private final Timer _updateTimer = new Timer();
	private final Timer _navigationTimer = new Timer();
	private final Timer _captureTimer = new Timer();

	/**
	 * Defines the PID gains that will be returned if there is an error.
	 */
	public static final double[] NAN_GAINS = new double[] { Double.NaN,
			Double.NaN, Double.NaN };

	// Define Amarino function control codes
	public static final char GET_RUDDER_FN = 'r';
	public static final char GET_THRUST_FN = 't';
	public static final char GET_TE_FN = 's';
	public static final char GET_ES_FN = 'e';
	public static final char SET_VELOCITY_FN = 'v';
	public static final char GET_DEPTH_FN = 'd';
	public static final char SET_SAMPLER_FN = 'q';
	public static final char GET_WATERCANARY_FN = 'w';
	public static final char GET_DO_FN = 'o';
	public static final char GET_MONITOR_FN = 'm';

	// Set timeout for asynchronous Amarino calls
	public static final int RESPONSE_TIMEOUT_MS = 250; // was 200 earlier

	// Status information
	final AtomicBoolean _isAutonomous = new AtomicBoolean(false);

	// Internal data structures for Amarino callbacks
	final Context _context;
	final List<String> _partialCommand = new ArrayList<String>(10);
	public static final double[] DEFAULT_TWIST = {0, 0, 0, 0, 0, 0}; 

	public final Object _connectionLock = new Object();
	public FileDescriptor _connection = null;
	public FileInputStream _connectionIn = null;
	public FileOutputStream _connectionOut= null;
	
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
	 * Hard-coded constants used in Yunde's controller and for new implementation of Arduino code.
	 * CONSTANTS FORMAT: range_min, range_max, servo_min, servo_max
	 */
	double[] r_PID = {2, 0, 3}; // Kp, Ki, Kd
	double [] t_PID = {5, 5, 5};
	public static final double CONST_THRUST = 0.5;

	/**
	 * Creates a new instance of the vehicle implementation. This function
	 * should only be used internally when the corresponding vehicle service is
	 * started and stopped.
	 * 
	 * @param context
	 *            the application context to use
	 * @param addr
	 *            the bluetooth address of the vehicle controller
	 */

	protected AirboatImpl(Context context) {
		_context = context;

		// Start a regular update function
		_updateTimer.scheduleAtFixedRate(_updateTask, 0, UPDATE_INTERVAL_MS);
	}

	/**
	 * Internal update function called at regular intervals to process command
	 * and control events.
	 * 
	 * @param dt
	 *            the elapsed time since the last update call (in seconds)
	 */
	private TimerTask _updateTask = new TimerTask() {
		// long _lastUpdateMs = 0;

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

			// Call Amarino with new velocities here
			// Yes, I know this looks silly, but Amarino doesn't handle doubles
			try {
				JSONArray velocity = new JSONArray();
				velocity.put(_velocities.dx());
				velocity.put(_velocities.dy());
				velocity.put(_velocities.dz());
				velocity.put(_velocities.drx());
				velocity.put(_velocities.dry());
				velocity.put(_velocities.drz());
				
				JSONObject object = new JSONObject();
				object.put("velocity", velocity);
				
				synchronized(_connectionLock) {
					if (_connection != null) {
						_connectionOut.write(object.toString().getBytes(US_ASCII));
					}
				}
			} catch (IOException e) {
				// TODO: Do something
			} catch (JSONException e) {
				// TODO: Do something
			}

			// Log velocities
			logger.info("VEL: " + _velocities);

			// Send velocities
			Twist vel = _velocities.clone();
			sendVelocity(vel);
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
		else
			return NAN_GAINS;
	}

	/**
	 * @see VehicleServer#setGains(int, double[])
	 */
	@Override
	public void setGains(int axis, double[] k) {
		if (axis == 5)
			r_PID = k.clone();
		else if (axis == 0)
			t_PID = k.clone();
		logger.info("SETGAINS: " + axis + " " + Arrays.toString(k));
	}
	/**
	 * Returns the current gyro readings
	 */
	public double[] getGyro()
	{
		return _gyroPhone.clone();
	}
	/**
	 * Function that maps a value between one range to a representative value in another range. Use for modifying servo commands
	 * to send to Arduino   
	 */
	public static double map(double x, double in_min, double in_max, double out_min, double out_max)
	{
	  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}
	
	public void setPhoneGyro(float[] gyroValues)
	{
		for (int i = 0; i < gyroValues.length; i++)
			_gyroPhone[i] =  (double) gyroValues[i];
	}
	
	/**
	 * @see AirboatCommand#isConnected()
	 */
	public boolean isConnected() {
		synchronized(_connectionLock) {
			return (_connection == null);
		}
	}
	/**
	 * Internal function used to set the connection object to the vehicle.
	 */
	protected void setConnection(ParcelFileDescriptor connection) {
		synchronized(_connectionLock) {
			_connection = connection.getFileDescriptor();
		}
	}

	/**
	 * Handles complete Arduino commands, once they are reassembled.
	 * 
	 * @param response
	 *            the arguments composing a response
	 */
	protected void onCommand(JSONObject response) {
		
		// Get a list of the keys in this response
		@SuppressWarnings("unchecked")
		Iterator<String> it = (Iterator<String>)response.keys();
		
		// Iterate over object keys to decipher commands
		try {
			while (it.hasNext()) {
				String key = it.next();
				JSONObject args = response.getJSONObject(key);
				
				if (key.equalsIgnoreCase("motor")) {
					logger.info("MOTOR[" + args.getInt("channel") + "]: " + args.getDouble("value"));
					break;
				} else if (key.equalsIgnoreCase("servo")) {
					logger.info("SERVO[" + args.getInt("channel") + "]: " + args.getDouble("value"));
					break;
				} else if (key.equalsIgnoreCase("sensor")) {
					String type = args.getString("type");
					int index = args.getInt("index");
					JSONArray vals = args.getJSONArray("values");
					
					// Decode the sensor reading
					SensorData reading = new SensorData();
					reading.channel = index;
					reading.type = SensorType.valueOf(type);
					
					reading.data = new double[vals.length()];
					for (int i = 0; i < vals.length(); ++i) {
						reading.data[i] = vals.getDouble(i);
					}
					
					// Log the sensor reading to file
					logger.info("SENSOR[" + index + "](" + type + "): " + vals.join(","));
				} else {
					Log.w(logTag, "Received unknown function type: " + key);
					break;
				}
			} 
		} catch (JSONException e) {
			Log.w(logTag, "Malformed response: " + response);
		}
	}

	/**
	 * Waits for incoming data from the device, parses it into JSON 
	 * and calls onCommand with it.
	 */
	public Runnable receiver = new Runnable() {
		@Override
		public void run() {
			// Allocate a read buffer
			byte[] data = new byte[512];

			// Run until connection is closed
			try {
				while (true) {
					synchronized(_connection) {
						_connectionIn.read(data);
						JSONObject command = new JSONObject(new String(data, US_ASCII));
						onCommand(command);
					}
				}
			} catch (JSONException e) {
				// TODO: log errors here
			} catch (IOException e) {
				// TODO: log errors here				
			}
		}
	};

	// TODO: Revert capture image to take images
	// This is a hack to support the water sampler until PID is working again.
	public synchronized byte[] captureImage(int width, int height) {
		// Call Amarino to fire sampler
		try {
			JSONObject object = new JSONObject();
			object.put("sampler", true);
			
			synchronized(_connectionLock) {
				if (_connection != null) {
					_connectionOut.write(object.toString().getBytes(US_ASCII));
				}
			}
		} catch (IOException e) {
			// TODO: Do something
		} catch (JSONException e) {
			// TODO: Do something
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
		Log.i(logTag, "Starting waypoints with " + controller + ": "
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
				VehicleController vc = AirboatController.STOP.controller;
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
							return;
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

		setAutonomous(false);
		setConnection(null);

		_updateTimer.cancel();
		_updateTimer.purge();
		
		_navigationTimer.cancel();
		_navigationTimer.purge();
		
		_captureTimer.cancel();
		_captureTimer.purge();
	}
}
