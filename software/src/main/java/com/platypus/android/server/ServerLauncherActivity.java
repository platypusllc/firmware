package com.platypus.android.server;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.widget.Toast;

/**
 * Launcher that starts Platypus Server whenever accessory is plugged in.
 * 
 * This is required because Android does not allow USB connection events to be
 * received by services directly. Instead, this activity is launched, which does
 * nothing but use the USB connection event to start the vehicle server.
 * 
 * Adapted from:
 * https://github.com/follower/android-background-service-usb-accessory
 * 
 * @author pkv
 */
public class ServerLauncherActivity extends Activity {
    private String TAG = ServerLauncherActivity.class.getName();

    /**
     * Called when the activity is first created.
     */
    @Override
    public void onCreate(Bundle savedInstanceState) {
	super.onCreate(savedInstanceState);

	Log.d(TAG, "Starting vehicle service launcher.");
	// TODO: remove this print
	Toast.makeText(this, "Platypus Control Board detected.",
		Toast.LENGTH_SHORT).show();

	Intent intent = new Intent(this, ServerService.class);
	intent.fillIn(getIntent(), 0);
	startService(intent);

	// See:
	//
	// <http://permalink.gmane.org/gmane.comp.handhelds.android.devel/154481>
	// &
	// <http://stackoverflow.com/questions/5567312/android-how-to-execute-main-fucntionality-of-project-only-by-clicking-on-the-ic/5567514#5567514>
	//
	// for combination of `Theme.NoDisplay` and `finish()` in
	// `onCreate()`/`onResume()`.
	//
	finish();

	Log.d(TAG, "Exiting vehicle service launcher.");
    }
}