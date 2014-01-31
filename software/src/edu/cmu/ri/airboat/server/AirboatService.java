package edu.cmu.ri.airboat.server;


import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.net.InetSocketAddress;
import java.text.SimpleDateFormat;
import java.util.Date;

import javax.measure.unit.NonSI;
import javax.measure.unit.SI;

import org.jscience.geography.coordinates.LatLong;
import org.jscience.geography.coordinates.UTM;
import org.jscience.geography.coordinates.crs.ReferenceEllipsoid;
import org.json.JSONException;
import org.json.JSONObject;

import robotutils.Pose3D;
import robotutils.Quaternion;
import android.app.Notification;
import android.app.NotificationManager;
import android.app.PendingIntent;
import android.app.Service;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.hardware.usb.UsbAccessory;
import android.hardware.usb.UsbManager;
import android.location.Criteria;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.net.wifi.WifiManager;
import android.net.wifi.WifiManager.WifiLock;
import android.os.Binder;
import android.os.Bundle;
import android.os.Debug;
import android.os.IBinder;
import android.os.ParcelFileDescriptor;
import android.os.PowerManager;
import android.os.PowerManager.WakeLock;
import android.os.StrictMode;
import android.util.Log;

import com.google.code.microlog4android.LoggerFactory;
import com.google.code.microlog4android.appender.FileAppender;
import com.google.code.microlog4android.config.PropertyConfigurator;
import com.google.code.microlog4android.format.PatternFormatter;

import edu.cmu.ri.crw.CrwNetworkUtils;
import edu.cmu.ri.crw.CrwSecurityManager;
import edu.cmu.ri.crw.data.Utm;
import edu.cmu.ri.crw.data.UtmPose;
import edu.cmu.ri.crw.udp.UdpVehicleService;

/**
 * Android Service to register sensor and Amarino handlers for Android.s
 * Contains a RosVehicleServer and a VehicleServer object.
 * 
 * @author pkv
 * @author kshaurya
 *
 */
public class AirboatService extends Service {
	private static final int SERVICE_ID = 11312;
	private static final String TAG = AirboatService.class.getName();
	private static final com.google.code.microlog4android.Logger logger = LoggerFactory.getLogger();
	
	// Default values for parameters
	private static final String DEFAULT_LOG_PREFIX = "airboat_";
	private static final int DEFAULT_UDP_PORT = 11411;
	final int GPS_UPDATE_RATE = 200; //in milliseconds
	
	// Intent fields definitions
	public static final String BD_ADDR = "BD_ADDR";
	public static final String UDP_REGISTRY_ADDR = "UDP_REGISTRY_ADDR";
	public static final String UPDATE_RATE = "UPDATE_RATE";
	
	// Binder object that receives interactions from clients.
    private final IBinder _binder = new AirboatBinder();
    
    // Reference to USB accessory
    private UsbManager mUsbManager;
    private UsbAccessory mUsbAccessory;
    private ParcelFileDescriptor mUsbDescriptor;
    
	// Flag indicating run status (Android has no way to query if a service is running)
	public static boolean isRunning = false;

	// Member parameters 
	private InetSocketAddress _udpRegistryAddr;
	
	// Objects implementing actual functionality
	private AirboatImpl _airboatImpl;
	private UdpVehicleService _udpService;

	// Lock objects that prevent the phone from sleeping
	private WakeLock _wakeLock = null;
    private WifiLock _wifiLock = null;
	
	// Logger that pipes log information for airboat classes to file
	private FileAppender _fileAppender; 
    
    // global variable to reference rotation vector values
    private float[] rotationMatrix = new float[9];

	/**
	 * Handles GPS updates by calling the appropriate update.
	 */
	private LocationListener locationListener = new LocationListener() {
        public void onStatusChanged(String provider, int status, Bundle extras) {}
        public void onProviderEnabled(String provider) {}
        public void onProviderDisabled(String provider) {}
        
        public void onLocationChanged(Location location) {
        	
        	// Convert from lat/long to UTM coordinates
        	UTM utmLoc = UTM.latLongToUtm(
        				LatLong.valueOf(location.getLatitude(), location.getLongitude(), NonSI.DEGREE_ANGLE), 
        				ReferenceEllipsoid.WGS84
        			);

        	// Convert to UTM data structure
        	Pose3D pose = new Pose3D(
        			utmLoc.eastingValue(SI.METER),
        			utmLoc.northingValue(SI.METER),
        			(location.hasAltitude() ? location.getAltitude() : 0.0),
        			(location.hasBearing() ? Quaternion.fromEulerAngles(0.0, 0.0, (90.0 - location.getBearing()) * Math.PI / 180.0) : Quaternion.fromEulerAngles(0, 0, 0)) 
        			);
        	Utm origin = new Utm(utmLoc.longitudeZone(), utmLoc.latitudeZone() > 'O');
        	UtmPose utm = new UtmPose(pose, origin);
        	        	
        	// Apply update using filter object
        	if (_airboatImpl != null) {
        		_airboatImpl.filter.gpsUpdate(utm, location.getTime());
        		logger.info("GPS: " + utmLoc + ", " 
        				+ utmLoc.longitudeZone() + utmLoc.latitudeZone() + ", " 
        				+ location.getAltitude() + ", " + location.getBearing());
        	}
        }
      };
    private final SensorEventListener rotationVectorListener = new SensorEventListener() {
		@Override
		public void onSensorChanged(SensorEvent event) {
			if (event.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR)
			{
			// TODO Auto-generated method stub
			SensorManager.getRotationMatrixFromVector(rotationMatrix, event.values);
			double yaw = Math.atan2(-rotationMatrix[5], -rotationMatrix[2]);
			
			if (_airboatImpl != null) {
				_airboatImpl.filter.compassUpdate(yaw, System.currentTimeMillis());
				logger.info("COMPASS: " +  yaw);
			}
			}
		}
		
		@Override
		public void onAccuracyChanged(Sensor sensor, int accuracy) {}
	};
    /**
     * UPDATE: 7/03/12 - Handles gyro updates by calling the appropriate update.
     */
    private final SensorEventListener gyroListener = new SensorEventListener() {
		@Override
		public void onSensorChanged(SensorEvent event) {
			// TODO Auto-generated method stub
			/* Convert phone coordinates to world coordinates. use magnetometer and accelerometer to get orientation
			 * Simple rotation is 90� clockwise about positive y axis. Thus, transformation is:
			// /  M[ 0]   M[ 1]   M[ 2]  \ / values[0] \ = gyroValues[0]
			// |  M[ 3]   M[ 4]   M[ 5]  | | values[1] | = gyroValues[1]
			// \  M[ 6]   M[ 7]   M[ 8]  / \ values[2] / = gyroValues[2]
			// 
			*/
			float[] gyroValues = new float[3];
			gyroValues[0] = rotationMatrix[0] * event.values[0] + rotationMatrix[1] * event.values[1] + rotationMatrix[2] * event.values[2];
			gyroValues[1] = rotationMatrix[3] * event.values[0] + rotationMatrix[4] * event.values[1] + rotationMatrix[5] * event.values[2];
			gyroValues[2] = rotationMatrix[6] * event.values[0] + rotationMatrix[7] * event.values[1] + rotationMatrix[8] * event.values[2];
			
			if (_airboatImpl != null)
				_airboatImpl.setPhoneGyro(gyroValues);
    		//Log.w("this app", "gyroValues with RV: " + Float.toString(gyroValues[0]) + "," + Float.toString(gyroValues[1]) + "," + Float.toString(gyroValues[2]));
    		//Log.e("this app", "event Values: " + Float.toString(event.values[0]) + "," + Float.toString(event.values[1]) + "," + Float.toString(event.values[2]));
		}
		@Override
		public void onAccuracyChanged(Sensor sensor, int accuracy) {}
	};

	/**
     * Class for clients to access.  Because we know this service always
     * runs in the same process as its clients, we don't deal with IPC.
     */
    public class AirboatBinder extends Binder {
        AirboatService getService() {
            return AirboatService.this;
        }
    }
	
	@Override
	public IBinder onBind(Intent intent) {
		return _binder;
	}
	
	@Override
	public void onCreate() {
		super.onCreate();
		
		// Disable all DNS lookups (safer for private/ad-hoc networks)
		CrwSecurityManager.loadIfDNSIsSlow();
		isRunning = true;
		
		// Disable strict-mode (TODO: remove this and use handlers)
		StrictMode.ThreadPolicy policy = new StrictMode.ThreadPolicy.Builder().permitAll().build();
		StrictMode.setThreadPolicy(policy);
		
        // Get USB Manager to handle USB accessories.
        mUsbManager = (UsbManager) getSystemService(Context.USB_SERVICE);
		
		// TODO: optimize this to allocate resources up here and handle multiple start commands
	}

	/**
	 * Access method to get underlying implementation of server functionality.
	 * @return An interface allowing high-level control of the boat.
	 */
	public AirboatImpl getServer() {
		return _airboatImpl;
	}

	
	/** 
     * Constructs a default filename from the current date and time.
     * @return the default filename for the current time.
     */
    private static String defaultLogFilename() {
        Date d = new Date();
        SimpleDateFormat sdf = new SimpleDateFormat("yyyyMMdd_hhmmss");
        return DEFAULT_LOG_PREFIX + sdf.format(d) + ".txt";
    }
    
    /**
     * Main service initialization: called whenever a request is
     * made to start the Airboat service.  
     * 
     * This is where the vehicle implementation is started, sensors are 
     * registered, and the update loop and RPC server are started.   
     */
	@Override
	public int onStartCommand(final Intent intent, int flags, int startId) {
		super.onStartCommand(intent, flags, startId);
		
		// Ignore startup requests that don't include an intent
		if (intent == null) {
			Log.e(TAG, "Started with null intent.");
			return Service.START_STICKY;
		}

		// Ignore startup requests without an accessory.
		if (!intent.hasExtra(UsbManager.EXTRA_ACCESSORY)) {
			Log.e(TAG, "Attempted to start without accessory.");
			return Service.START_STICKY;
		}
		
		// Ensure that we do not reinitialize if not necessary
		if (_airboatImpl != null || _udpService != null) {
			Log.w(TAG, "Attempted to start while running.");
			return Service.START_STICKY;
		}

		// start tracing to "/sdcard/trace_crw.trace"
		//Debug.startMethodTracing("trace_crw");
		
		// Get context (used for system functions)
		Context context = getApplicationContext();
		
		// Set up logging format to include time, tag, and value
        PropertyConfigurator.getConfigurator(this).configure();		    
		PatternFormatter formatter = new PatternFormatter(); 
		formatter.setPattern("%r %d %m %T");
		
		// Set up and register data logger to a date-stamped file
		String logFilename = defaultLogFilename();
		_fileAppender = new FileAppender();
		_fileAppender.setFileName(logFilename);
		_fileAppender.setAppend(true);
		_fileAppender.setFormatter(formatter);
		try {
			_fileAppender.open();
		} catch (IOException e) {
			Log.w(TAG, "Failed to open data log file: " + logFilename, e);
			sendNotification("Failed to open log: " + e.getMessage());
		}
	    logger.addAppender(_fileAppender);

		// Hook up to necessary Android sensors
        SensorManager sm; 
        sm = (SensorManager)getSystemService(SENSOR_SERVICE);
        Sensor gyro = sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        sm.registerListener(gyroListener, gyro, SensorManager.SENSOR_DELAY_NORMAL);
        Sensor rotation_vector = sm.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
        sm.registerListener(rotationVectorListener, rotation_vector, SensorManager.SENSOR_DELAY_NORMAL);
        
        // Hook up to the GPS system
        LocationManager gps = (LocationManager)getSystemService(Context.LOCATION_SERVICE);
        Criteria c = new Criteria();
        c.setAccuracy(Criteria.ACCURACY_FINE);
        c.setPowerRequirement(Criteria.NO_REQUIREMENT);
        String provider = gps.getBestProvider(c, false);
        gps.requestLocationUpdates(provider, GPS_UPDATE_RATE, 0, locationListener);
		
        // Create an intent filter to listen for device disconnections
        IntentFilter filter = new IntentFilter(UsbManager.ACTION_USB_ACCESSORY_DETACHED);
        registerReceiver(_usbStatusReceiver, filter);
        
        // Connect to control board.
        // (Assume that we can only be launched by the LauncherActivity which
        // provides a handle to the accessory.)
        mUsbAccessory = (UsbAccessory) intent.getParcelableExtra(UsbManager.EXTRA_ACCESSORY);
        mUsbDescriptor = mUsbManager.openAccessory(mUsbAccessory);
        if (mUsbDescriptor == null) {
            // If the accessory fails to connect, terminate service.
            Log.e(TAG, "Failed to open accessory.");
            stopSelf();
            return Service.START_STICKY;
        }
        
        // Create writer for output over USB
        PrintWriter usbWriter = new PrintWriter(new OutputStreamWriter(
                new FileOutputStream(mUsbDescriptor.getFileDescriptor())));
        final BufferedReader usbReader = new BufferedReader(new InputStreamReader(
                new FileInputStream(mUsbDescriptor.getFileDescriptor())));
		
        // Create a filter to listen for obstacle avoidance instructions
        IntentFilter obstacleFilter = new IntentFilter();
        obstacleFilter.addAction(AirboatImpl.OBSTACLE);
		
		// Create the data object
		_airboatImpl = new AirboatImpl(this, usbWriter);
		new Thread(new Runnable() {
			@Override
			public void run() {
				// Start a loop to receive data from accessory.
		        try {
		            while (true) {
		                // Handle this response
		            	String line = usbReader.readLine();
		            	try {
		            		_airboatImpl.onCommand(new JSONObject(line));
		            	} catch (JSONException e) {
		            		Log.w(TAG, "Failed to parse response '" + line + "'.", e);
		            	}
		            }
		        } catch (IOException e) {
		            Log.d(TAG, "Accessory connection closed.", e);
		        }
		        
		        try { usbReader.close(); } catch (IOException e) {}
			}
		});
		
		// Start up UDP vehicle service in the background
		new Thread(new Runnable() {
			@Override
			public void run() {
				try {
					// Create a UdpVehicleService to expose the data object
					_udpService = new UdpVehicleService(DEFAULT_UDP_PORT, _airboatImpl);
					
					// If given a UDP registry parameter, add registry to service
					String udpRegistryStr = intent.getStringExtra(UDP_REGISTRY_ADDR);
					_udpRegistryAddr = CrwNetworkUtils.toInetSocketAddress(udpRegistryStr);
					if (_udpRegistryAddr != null) {
						_udpService.addRegistry(_udpRegistryAddr);
					} else {
						Log.w(TAG, "Unable to parse '" + udpRegistryStr + "' into UDP address.");
					}
				} catch (Exception e) {
					Log.e(TAG, "UdpVehicleService failed to launch", e);
					sendNotification("UdpVehicleService failed: " + e.getMessage());
					stopSelf();
					return;
				}
			}
		}).start();
		
		// Log the velocity gains before starting the service
		new Thread(new Runnable() {
			@Override
			public void run() {
				do {
					try { Thread.sleep(5000); } catch (InterruptedException ex) { }
					
					double[] velGains;
					if (_airboatImpl != null) {
						velGains = _airboatImpl.getGains(0);
						logger.info("PIDGAINS: " + "0 " + velGains[0] + "," + velGains[1] + "," + velGains[2]);
					}
					
					if (_airboatImpl != null) {
						velGains = _airboatImpl.getGains(5);
						logger.info("PIDGAINS: " + "5 " + velGains[0] + "," + velGains[1] + "," + velGains[2]);
					}
					
				} while (_airboatImpl == null);
			}
		}).start();

		// This is now a foreground service
		{
			// Set up the icon and ticker text
			int icon = R.drawable.icon; // TODO: change this to notification icon
			CharSequence tickerText = "Running normally.";
			long when = System.currentTimeMillis();
			
			// Set up the actual title and text
			CharSequence contentTitle = "Airboat Server";
			CharSequence contentText = tickerText;
			Intent notificationIntent = new Intent(this, AirboatActivity.class);
			PendingIntent contentIntent = PendingIntent.getActivity(this, 0, notificationIntent, 0);
		
			// Add a notification to the menu
			Notification notification = new Notification(icon, tickerText, when);
			notification.setLatestEventInfo(context, contentTitle, contentText, contentIntent);
		    startForeground(SERVICE_ID, notification);
		}
		
		// Prevent phone from sleeping or turning off wifi
		{
			// Acquire a WakeLock to keep the CPU running
			PowerManager pm = (PowerManager)context.getSystemService(Context.POWER_SERVICE);
			_wakeLock = pm.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "AirboatWakeLock");
			_wakeLock.acquire();
			
			// Acquire a WifiLock to keep the phone from turning off wifi
			WifiManager wm = (WifiManager)context.getSystemService(Context.WIFI_SERVICE);
			_wifiLock = wm.createWifiLock(WifiManager.WIFI_MODE_FULL_HIGH_PERF, "AirboatWifiLock");
			_wifiLock.acquire();
		}
		
		// Indicate that the service should not be stopped arbitrarily
		Log.i(TAG,"AirboatService started.");
		return Service.START_STICKY;
	}
	
	/**
	 * Called when there are no longer any consumers of the service and 
	 * stopService has been called.
	 * 
	 * This is where the RPC server and update loops are killed, the sensors 
	 * are unregistered, and the current vehicle implementation is unhooked 
	 * from all of its callbacks and discarded (allowing safe spawning of a new
	 * implementation when the service is restarted).
	 */
	@Override
	public void onDestroy() {
		// Stop tracing to "/sdcard/trace_crw.trace"
		Debug.stopMethodTracing();
		
		// Shutdown the vehicle services
		if (_udpService != null) {
			try {
				_udpService.shutdown();
			} catch (Exception e) {
				Log.e(TAG, "UdpVehicleService shutdown error", e);
			}
			_udpService = null;
		}
		
		// Release locks on wifi and CPU
		if (_wakeLock != null) {
			_wakeLock.release();
		}
		if (_wifiLock != null) {
			_wifiLock.release();
		}
		
		// Disconnect from the Android sensors
        SensorManager sm; 
        sm = (SensorManager)getSystemService(SENSOR_SERVICE);
        sm.unregisterListener(gyroListener);
        sm.unregisterListener(rotationVectorListener);
		
        // Disconnect from GPS updates
        LocationManager gps;
        gps = (LocationManager)getSystemService(Context.LOCATION_SERVICE);
        gps.removeUpdates(locationListener);
        
		// Disconnect the data object from this service
		if (_airboatImpl != null) {
			try { mUsbDescriptor.close(); }  catch (IOException e) {}
			_airboatImpl.setConnected(false);
			_airboatImpl.shutdown();
			_airboatImpl = null;
		}

		// Remove the data log (a new one will be created on restart)
		if (_fileAppender != null) {
			try {
				_fileAppender.close();
			} catch (IOException e) {
				Log.e(TAG, "Data log shutdown error", e);
			}
		}

		// Disable this as a foreground service
		stopForeground(true);
		
		Log.i(TAG, "AirboatService stopped.");
		isRunning = false;
		super.onDestroy();
	}
	
	public void sendNotification(CharSequence text) {
		String ns = Context.NOTIFICATION_SERVICE;
		NotificationManager notificationManager = (NotificationManager)getSystemService(ns);

		// Set up the icon and ticker text
		int icon = R.drawable.icon; // TODO: change this to notification icon
		CharSequence tickerText = text;
		long when = System.currentTimeMillis();
	
		// Set up the actual title and text
		Context context = getApplicationContext();
		CharSequence contentTitle = "Airboat Server";
		CharSequence contentText = text;
		Intent notificationIntent = new Intent(this, AirboatService.class);
		PendingIntent contentIntent = PendingIntent.getActivity(this, 0, notificationIntent, 0);
	
		Notification notification = new Notification(icon, tickerText, when);
		notification.setLatestEventInfo(context, contentTitle, contentText, contentIntent);
		notification.flags |= Notification.FLAG_AUTO_CANCEL;
		
		notificationManager.notify(SERVICE_ID, notification);
	}
	
	/**
     * Listen for disconnection events for accessory and close connection.
     */
    BroadcastReceiver _usbStatusReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {

            // Retrieve the device that was just disconnected.
            UsbAccessory accessory = (UsbAccessory) intent
                    .getParcelableExtra(UsbManager.EXTRA_ACCESSORY);

            // Check if this accessory matches the one we have open.
            if (mUsbAccessory.equals(accessory)) {
                try {
                    // Close the descriptor for our accessory.
                    // (This triggers server shutdown.)
                    mUsbDescriptor.close();
                    Log.e(TAG, "Closing accessory.");
                } catch (IOException e) {
                    Log.w(TAG, "Failed to close accessory cleanly.", e);
                }
                
                stopSelf();
            }
        }
    };
}
