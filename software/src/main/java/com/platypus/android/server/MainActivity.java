
package com.platypus.android.server;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuItem;

public class MainActivity extends Activity {

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
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
    }

    /**
     * Inflate the menu based on the defined XML file.
     * 
     * @return True if the menu was handled within this function. 
     */
    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(com.platypus.android.server.R.menu.main, menu);
        return true;
    }

    /**
     * Handles menu events by dispatching them according to their ids.
     * 
     * @return True if the menu item was handled within this function. 
     */
    @Override
    public boolean onOptionsItemSelected(MenuItem item)
    {
        // Handle item selection
        switch (item.getItemId()) {
            case R.id.action_settings:
                startActivity(new Intent(this, SettingsActivity.class));
                return true;
            case R.id.action_launch:
                startActivity(new Intent(this, LauncherActivity.class));
                return true;
            default:
                return super.onOptionsItemSelected(item);
        }
    }

}
