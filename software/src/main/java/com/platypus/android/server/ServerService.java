/**
 * TODO: Insert license information here
 */

package com.platypus.android.server;

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

import com.platypus.android.server.transport.UdpTransport;
import com.platypus.android.server.transport.WebSocketTransport;
import com.platypus.protobuf.PlatypusCommand;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;

/**
 * Implements a Platypus Vehicle Server.
 * 
 * @author pkv
 */
public class ServerService extends IntentService {
    private static final String TAG = ServerService.class.getName();
    private static final int ONGOING_NOTIFICATION_ID = 35332;

    private UsbManager usbManager_;
    private UsbAccessory usbAccessory_;
    private ParcelFileDescriptor usbDescriptor_;
    private JsonWriter usbWriter_;

    private SharedPreferences prefs_;

    public ServerService() {
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
        usbManager_ = (UsbManager) getSystemService(Context.USB_SERVICE);

        // Listen for shared preference changes
        prefs_ = PreferenceManager.getDefaultSharedPreferences(this);
        prefs_.registerOnSharedPreferenceChangeListener(prefListener_);
        // TODO: set initial values for these system settings

        // Create an intent filter to listen for device disconnections
        IntentFilter filter = new IntentFilter(UsbManager.ACTION_USB_ACCESSORY_DETACHED);
        registerReceiver(usbReceiver_, filter);
        
        // Create a UDP server socket
        new UdpTransport(11311, serverReceiver_);
        
        // Create a TCP server WebSocket
        new WebSocketTransport(11411, serverReceiver_);
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
        usbAccessory_ = (UsbAccessory) intent.getParcelableExtra(UsbManager.EXTRA_ACCESSORY);
        usbDescriptor_ = usbManager_.openAccessory(usbAccessory_);
        if (usbDescriptor_ == null) {
            // If the accessory fails to connect, terminate service.
            Log.e(TAG, "Failed to open accessory.");
            return;
        }

        // Get input and output streams
        JsonReader usbReader = new JsonReader(new InputStreamReader(
                new FileInputStream(usbDescriptor_.getFileDescriptor())));
        usbWriter_ = new JsonWriter(new OutputStreamWriter(
                new FileOutputStream(usbDescriptor_.getFileDescriptor())));

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

        // TODO: is this actually necessary? The only way to get here is to
        // close the descriptor
        // Shutdown the connection to the accessory.
        try {
            usbDescriptor_.close();
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

        // If for any reason the device is not shutdown, do it now.
        try {
            usbDescriptor_.close();
        } catch (IOException e) {
            Log.w(TAG, "Failed to close accessory cleanly.", e);
        }

        // Disconnect intent filter to listen for device disconnections
        unregisterReceiver(usbReceiver_);

        // Unregister the shared preferences
        prefs_.unregisterOnSharedPreferenceChangeListener(prefListener_);

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

            if ("Sensor".equalsIgnoreCase(name)) {
                // TODO: respond with something
            } else if ("Motor".equalsIgnoreCase(name)) {
                // TODO: respond with something
            } else {
                Log.w(TAG, "Unknown field '" + name + "'.");
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

    Transport.Receiver serverReceiver_ = new Transport.Receiver() {
        public void receive(PlatypusCommand command, Transport transport) {
            // TODO Auto-generated method stub
        }
    };
    
    /**
     * Listen for disconnection events for accessory and close connection.
     */
    BroadcastReceiver usbReceiver_ = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {

            // Retrieve the device that was just disconnected.
            UsbAccessory accessory = (UsbAccessory) intent
                    .getParcelableExtra(UsbManager.EXTRA_ACCESSORY);

            // Check if this accessory matches the one we have open.
            if (usbAccessory_.equals(accessory)) {
                try {
                    // Close the descriptor for our accessory.
                    // (This triggers server shutdown.)
                    usbDescriptor_.close();
                    Log.e(TAG, "Closing accessory.");
                } catch (IOException e) {
                    Log.w(TAG, "Failed to close accessory cleanly.", e);
                }
            }
        }
    };
}
