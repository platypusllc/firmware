/**
 * TODO: Insert license information here
 */

package com.platypus.android.server;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.util.List;
import java.util.UUID;

import android.app.IntentService;
import android.app.Notification;
import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.SharedPreferences;
import android.content.SharedPreferences.OnSharedPreferenceChangeListener;
import android.hardware.usb.UsbAccessory;
import android.hardware.usb.UsbManager;
import android.os.ParcelFileDescriptor;
import android.preference.PreferenceManager;
import android.util.JsonReader;
import android.util.JsonWriter;
import android.util.Log;

import com.madara.KnowledgeBase;
import com.madara.transport.TransportSettings;
import com.madara.transport.TransportType;
import com.platypus.android.server.module.CameraModule;

/**
 * Implements a Platypus Vehicle Server.
 * 
 * @author pkv
 */
public class VehicleService extends IntentService {
    private static final String TAG = VehicleService.class.getName();
    private static final int ONGOING_NOTIFICATION_ID = 35332;

    private UsbManager mUsbManager;
    private UsbAccessory mUsbAccessory;
    private ParcelFileDescriptor mUsbDescriptor;
    protected JsonWriter mUsbWriter;

    private SharedPreferences mPrefs;
    private List<VehicleModule> mModules;

    protected KnowledgeBase mKnowledge;
    
    public VehicleService() {
        super("Platypus Server");
    }
    
    /**
     * Called by the system when the service is first created.
     */
    @Override
    public void onCreate() {

        // Defer to superclass
        super.onCreate();

        // Get USB Manager to handle USB accessories.
        mUsbManager = (UsbManager) getSystemService(Context.USB_SERVICE);

        // Listen for shared preference changes
        mPrefs = PreferenceManager.getDefaultSharedPreferences(this);
        mPrefs.registerOnSharedPreferenceChangeListener(prefListener_);

        // Create an intent filter to listen for device disconnections
        IntentFilter filter = new IntentFilter(UsbManager.ACTION_USB_ACCESSORY_DETACHED);
        registerReceiver(usbStatusReceiver_, filter);
        
        // MADARA configuration and setup.
        {
			// Create transport settings for a multicast transport
			TransportSettings settings = new TransportSettings();
			settings.setHosts(new String[]{mPrefs.getString(
					"MULTICAST_ADDRESS", "239.255.0.1:4150")});
			settings.setType(TransportType.MULTICAST_TRANSPORT);

			// Create a knowledge base with the multicast transport settings
			mKnowledge = new KnowledgeBase(mPrefs.getString("NAME", "Lutra"
					+ UUID.randomUUID()), settings);
        }
        
        // Create the camera module
        mModules.add(new CameraModule());
    }

    /*
     * (non-Javadoc)
     * @see android.app.IntentService#onHandleIntent(android.content.Intent)
     */
    @Override
    protected void onHandleIntent(Intent intent) {

        // Immediately register the server as a Foreground Service. This
        // prevents android from attempting to kill the service unless it is
        // critically low on resources.
        Intent notificationIntent = new Intent(this, MainActivity.class);
        PendingIntent contentIntent = PendingIntent.getActivity(this, 0, notificationIntent, 0);
        Notification foreground_notification = new Notification.Builder(this)
                .setSmallIcon(R.drawable.ic_stat_platypus_logo_transparent)
                .setContentTitle("Platypus Server")
                .setContentText("Connected to vehicle.")
                .setTicker("Detected Platypus Vehicle.")
                .setContentIntent(contentIntent).build();
        startForeground(ONGOING_NOTIFICATION_ID, foreground_notification);

        // Connect to control board.
        // (Assume that we can only be launched by the LauncherActivity which
        // provides a handle to the accessory.)
        mUsbAccessory = (UsbAccessory) intent.getParcelableExtra(UsbManager.EXTRA_ACCESSORY);
        mUsbDescriptor = mUsbManager.openAccessory(mUsbAccessory);
        if (mUsbDescriptor == null) {
            // If the accessory fails to connect, terminate service.
            Log.e(TAG, "Failed to open accessory.");
            return;
        }

        // Get input and output streams
        JsonReader usbReader = new JsonReader(new InputStreamReader(
                new FileInputStream(mUsbDescriptor.getFileDescriptor())));
        mUsbWriter = new JsonWriter(new OutputStreamWriter(
                new FileOutputStream(mUsbDescriptor.getFileDescriptor())));

        // Start all vehicle modules
        for (VehicleModule module : mModules) {
            module.start(this);
        }
        
        // Start a loop to receive data from accessory.
        try {
            while (true) {
                // Handle this response
                usbReader.beginObject();
                onAccessoryReceive(usbReader);
                usbReader.endObject();
            }
        } catch (IOException e) {
            Log.d(TAG, "Accessory connection closed.", e);
        }
        
        // Stop all vehicle modules
        for (VehicleModule module : mModules) {
            module.stop();
        }

        // TODO: is this actually necessary? The only way to get here is to
        // close the descriptor
        // Shutdown the connection to the accessory.
        try {
            mUsbDescriptor.close();
        } catch (IOException e) {
            Log.w(TAG, "Failed to close accessory cleanly.", e);
        }

        // Unregister as a foreground service and remove notification.
        stopForeground(true);
    }

    /**
     * Called by the system when the service is first created.
     */
    @Override
    public void onDestroy() {

    	// MADARA shutdown.
    	{
            mKnowledge.free();
            mKnowledge = null;
    	}
    	
        // If for any reason the device is not shutdown, do it now.
        try {
            mUsbDescriptor.close();
        } catch (IOException e) {
            Log.w(TAG, "Failed to close accessory cleanly.", e);
        }

        // Disconnect intent filter to listen for device disconnections
        unregisterReceiver(usbStatusReceiver_);

        // Unregister the shared preferences
        mPrefs.unregisterOnSharedPreferenceChangeListener(prefListener_);

        // Defer to superclass
        super.onDestroy();
    }

    /**
     * Handle responses from the USB accessory.
     * 
     * @param response Parsed JSON object containing response from USB
     *            accessory.
     */
    public void onAccessoryReceive(JsonReader reader) throws IOException {
        while (reader.hasNext()) {
            String name = reader.nextName();

            synchronized (mModules) {
                for (VehicleModule module : mModules) {
                    if (module.onAccessoryReceive(name, reader)) {
                        break;
                    }
                }
            }
        }
    }

    /**
     * Listens for changes in the server preferences, and immediately applies
     * them to the server.
     */
    OnSharedPreferenceChangeListener prefListener_ = new OnSharedPreferenceChangeListener() {
        public void onSharedPreferenceChanged(
                SharedPreferences sharedPreferences, String key) {
            // TODO: When android supports java 1.7 compliance, change this to
            // 'switch'.

            // Check for the key that changed and make the appropriate change to
            // the server
            if (key.equalsIgnoreCase("REGISTRY_IP")) {
                // TODO: Update based on new preferences
            } else if (key.equalsIgnoreCase("VEHICLE_TYPE")) {
                // TODO: Update based on new preferences
            } else if (key.equalsIgnoreCase("FAILSAFE_IP")) {
                // TODO: Update based on new preferences
            } else {
                Log.d(TAG, "Unknown preference '" + key + "' changed.");
            }
        }
    };
    
    /**
     * Listen for disconnection events for accessory and close connection.
     */
    BroadcastReceiver usbStatusReceiver_ = new BroadcastReceiver() {
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
            }
        }
    };
}
