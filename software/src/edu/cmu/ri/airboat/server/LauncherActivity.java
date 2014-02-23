package edu.cmu.ri.airboat.server;

import android.app.Activity;
import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.usb.UsbAccessory;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.util.Log;
import android.widget.Toast;

/**
 * Launcher that starts Server if accessory is plugged in. This
 * is required because Android does not allow USB connection events to be
 * received by services directly. Instead, this activity is launched, which does
 * nothing but use the USB connection event to start the vehicle server.
 * 
 * @see <a href="https://github.com/follower/android-background-service-usb-accessory">Android Background Service - USB accessory</a>
 * @author pkv
 */
public class LauncherActivity extends Activity {
    private String TAG = LauncherActivity.class.getName();
    private static final String ACTION_USB_PERMISSION = "com.platypus.android.server.USB_PERMISSION";

    /**
     * Called when the activity is first created.
     * 
     * @param savedInstanceState If the activity is being re-initialized after
     *            previously being shut down then this Bundle contains the data
     *            it most recently supplied in onSaveInstanceState(Bundle).
     *            <b>Note: Otherwise it is null.</b>
     */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        
        // Defer to superclass
        super.onCreate(savedInstanceState);
        
        Log.d(TAG, "Starting vehicle service launcher.");

        // Register a listener for USB permission events.
        // (This is cleaned up when the launcher is destroyed, but we might need
        // it if we have to search for an accessory.)
        IntentFilter filter = new IntentFilter(ACTION_USB_PERMISSION);
        registerReceiver(usbReceiver_, filter);
        
        // Request permission for ANY connected devices.
        UsbManager usbManager = (UsbManager) getSystemService(Context.USB_SERVICE);
        UsbAccessory[] usbAccessoryList = usbManager.getAccessoryList();

        if (usbAccessoryList != null && usbAccessoryList.length > 0) {
            // TODO: only detect Platypus Hardware!
            // At the moment, just use the first accessory (only one is
            // supported in android right now).
            PendingIntent permissionIntent = PendingIntent.getBroadcast(this, 0, new Intent(
                    ACTION_USB_PERMISSION), 0);
            usbManager.requestPermission(usbAccessoryList[0], permissionIntent);
        } else {
            Log.d(TAG, "Exiting vehicle service launcher: No devices found.");
            Toast.makeText(this, "No devices found.", Toast.LENGTH_SHORT).show();
            finish();
        }
    }
    
    @Override
    protected void onDestroy() {
        // Unregister the receiver for USB permission responses.
        unregisterReceiver(usbReceiver_);
        
        // Defer to superclass
        super.onDestroy();
    }

    /**
     * Waits for the return from a USB permission request. If the request is
     * granted, it starts the Platypus Server, if not, it simply ends the
     * Launcher activity.
     */
    private final BroadcastReceiver usbReceiver_ = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {

            // First, check that this permission response is actually for us.
            String action = intent.getAction();
            if (ACTION_USB_PERMISSION.equals(action)) {
                synchronized (this) {
                    // Get the accessory to which we are responding.
                    UsbAccessory accessory = intent.getParcelableExtra(UsbManager.EXTRA_ACCESSORY);

                    // Permission was granted, if the device exists: open it.
                    if (intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)) {
                        if (accessory != null) {
                            Intent server_intent = new Intent(LauncherActivity.this,
                                    AirboatService.class);
                            server_intent.fillIn(intent, 0);
                            server_intent.fillIn(getIntent(), 0);
                            startService(server_intent);
                            Log.d(TAG, "Exiting vehicle service launcher.");
                        } else {
                            // This is weird, we got permission, but to which
                            // device?
                            Log.w(TAG, "Exiting vehicle service launcher: No device returned.");
                        }
                    }
                    // Permission was not granted, don't open anything.
                    else {
                        Log.d(TAG, "Exiting vehicle service launcher: Permission denied.");
                    }

                    finish();
                }
            }
        }
    };
}
