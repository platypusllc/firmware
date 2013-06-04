package edu.cmu.ri.airboat.server;

import java.io.IOException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;
import java.util.concurrent.atomic.AtomicBoolean;

import javax.measure.unit.NonSI;
import javax.measure.unit.SI;

import org.jscience.geography.coordinates.LatLong;
import org.jscience.geography.coordinates.UTM;
import org.jscience.geography.coordinates.crs.ReferenceEllipsoid;

import robotutils.Pose3D;
import android.app.Activity;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.SharedPreferences;
import android.content.SharedPreferences.Editor;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Handler;
import android.text.Editable;
import android.text.TextWatcher;
import android.util.Log;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.ArrayAdapter;
import android.widget.AutoCompleteTextView;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;
import at.abraxas.amarino.AmarinoIntent;
import edu.cmu.ri.airboat.server.AirboatFailsafeService.AirboatFailsafeIntent;
import edu.cmu.ri.crw.CrwNetworkUtils;
import edu.cmu.ri.crw.data.Utm;
import edu.cmu.ri.crw.data.UtmPose;

public class AirboatActivity extends Activity {
	private static final String logTag = AirboatActivity.class.getName();
	
	public static final String PREFS_PRIVATE = "PREFS_PRIVATE";
	public static final String KEY_MASTER_URI = "KEY_MASTER_URI";
	public static final String KEY_BT_ADDR = "KEY_BT_ADDR";
	public static final String KEY_FAILSAFE_ADDR = "KEY_FAILSAFE_ADDR";
	public static final String OBSTACLE_DATA = "OBSTACLE_DATA";

	private BroadcastReceiver _amarinoReceiver;
	private UtmPose _homePosition = new UtmPose();
	
	/** Called when the activity is first created. */
    @Override
	public void onCreate(Bundle savedInstanceState) {

    	// Create the "main" layout from the included XML file 
    	super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
        
        // Register handler for Bluetooth address box
        final AutoCompleteTextView connectAddress = (AutoCompleteTextView)findViewById(R.id.ConnectAddress);
        connectAddress.setThreshold(1);
        connectAddress.addTextChangedListener(new TextWatcher() {
        	
        	public void onTextChanged(CharSequence s, int start, int before, int count) {}
			public void beforeTextChanged(CharSequence s, int start, int count, int after) {}
			
			public void afterTextChanged(Editable s) {
				
				// Remove all ':' and invalid chars from string
				String clean = s.toString().replaceAll("[:[^[\\d[a-f][A-F]]]]", "");
				
				// Insert them back in correct locations
				StringBuffer str = new StringBuffer(clean);
				if (str.length() > 2)
					str.insert(2, ':');
				if (str.length() > 5)
					str.insert(5, ':');
				if (str.length() > 8)
					str.insert(8, ':');
				if (str.length() > 11)
					str.insert(11, ':');
				if (str.length() > 14)
					str.insert(14, ':');
				if (str.length() > 17)
					str.delete(17, str.length());
				
				// If something changed, update string
				if (!str.toString().equals(s.toString()))
					s.replace(0, s.length(), str.toString().toUpperCase());
			}
		});
        
        // Create a filter that listens to Amarino connection events
        IntentFilter amarinoFilter = new IntentFilter();
        amarinoFilter.addAction(AmarinoIntent.ACTION_CONNECTED_DEVICES);
        amarinoFilter.addAction(AmarinoIntent.ACTION_CONNECTED);
        amarinoFilter.addAction(AmarinoIntent.ACTION_DISCONNECTED);
        
		// Create a listener to update the connected devices autocomplete list
        final ArrayAdapter<String> adapter = new ArrayAdapter<String>(this, android.R.layout.simple_dropdown_item_1line);
        connectAddress.setAdapter(adapter);
        _amarinoReceiver = new BroadcastReceiver() {
			
			@Override
			public void onReceive(Context context, Intent intent) {
				// Check for events indicating connection or disconnection
				if (intent.getAction().equals(AmarinoIntent.ACTION_CONNECTED)) {
					final String address = intent.getStringExtra(AmarinoIntent.EXTRA_DEVICE_ADDRESS);
					connectAddress.post(new Runnable() {
						public void run() {
							adapter.add(address);
							adapter.notifyDataSetChanged();
						}
					});
				} else if (intent.getAction().equals(AmarinoIntent.ACTION_DISCONNECTED)) {
					final String address = intent.getStringExtra(AmarinoIntent.EXTRA_DEVICE_ADDRESS);
					connectAddress.post(new Runnable() {
						public void run() {
							adapter.remove(address);
							adapter.notifyDataSetChanged();
						}
					});					
				} else if (intent.getAction().equals(AmarinoIntent.ACTION_CONNECTED_DEVICES)) {
					final String[] devices = intent.getStringArrayExtra(AmarinoIntent.EXTRA_CONNECTED_DEVICE_ADDRESSES);
					connectAddress.post(new Runnable() {
						public void run() {
							adapter.clear();
							if (devices != null)
								for (String device : devices) 
									adapter.add(device);
							adapter.notifyDataSetChanged();
						}
					});
				}
			}
		};
		registerReceiver(_amarinoReceiver, amarinoFilter);
		sendBroadcast(new Intent(AmarinoIntent.ACTION_GET_CONNECTED_DEVICES));
        
		// Register handler for URI master that changes the color of the URI
		// if a valid ROS core seems to be reached.
		// TODO: Move this to its own class!
		final AutoCompleteTextView masterAddress = (AutoCompleteTextView)findViewById(R.id.MasterAddress);
		masterAddress.addTextChangedListener(new TextWatcher() {
			
			final Handler handler = new Handler();
			final AtomicBoolean _isUpdating = new AtomicBoolean(false);
			final AtomicBoolean _isUpdated = new AtomicBoolean(false);
			
			final class TextUpdate extends AsyncTask<Void, Void, Integer> {
				private String text;
				
				@Override
				protected void onPreExecute() {
					text = masterAddress.getText().toString();
				}
				
				@Override
				protected Integer doInBackground(Void... urls) {
					int textBkgnd = 0xAAAA0000;
					
					_isUpdated.set(true);
					_isUpdating.set(true);
					
					try {
						// Try to open the host name in the text box, 
						// if it succeeds, change color accordingly
						InetSocketAddress addr = CrwNetworkUtils.toInetSocketAddress(text);
						
						if (addr != null && addr.getAddress().isReachable(500))
							textBkgnd = 0xAA00AA00;
				    } catch (IOException e) {}
				    
				    return textBkgnd;
				}

				@Override
				protected void onPostExecute(Integer result) {
					masterAddress.setBackgroundColor(result);

					// Immediately reschedule if out of date, otherwise delay
				    if (!_isUpdated.get()) {
				    	new TextUpdate().execute((Void[])null);
				    } else {
				    	handler.postDelayed(new Runnable() {
							@Override
							public void run() {
								new TextUpdate().execute((Void[])null);
							}
						}, 2000);
				    }
				    
				    // In any case, we are now done updating
				    _isUpdating.set(false);
				}
			};
			
			@Override
			public void onTextChanged(CharSequence s, int start, int before, int count) {}
			
			@Override
			public void beforeTextChanged(CharSequence s, int start, int count, int after) {}
			
			@Override
			public void afterTextChanged(final Editable s) {
				
				_isUpdated.set(false);
				
				// If an update isn't already running, start one up
				if (!_isUpdating.get()) {
					new TextUpdate().execute((Void[])null);
				}
			}
		});
		
        // Register handler for server toggle button
        final ToggleButton connectToggle = (ToggleButton)findViewById(R.id.ConnectToggle);
        connectToggle.setOnClickListener(new OnClickListener() {
        	
			@Override
			public void onClick(View v) {
				// Don't allow re-clicking until the service status updates
				connectToggle.setEnabled(false);
				
				// Create an intent to properly start the vehicle server
				Intent intent = new Intent(AirboatActivity.this, AirboatService.class);
    			intent.putExtra(AirboatService.BD_ADDR, connectAddress.getText().toString());
    			intent.putExtra(AirboatService.UDP_REGISTRY_ADDR, masterAddress.getText().toString());
    			
				// Save the current BD addr and master URI
				SharedPreferences prefs = getSharedPreferences(PREFS_PRIVATE, Context.MODE_PRIVATE);
				Editor prefsPrivateEditor = prefs.edit();
				prefsPrivateEditor.putString(KEY_BT_ADDR, connectAddress.getText().toString());
				prefsPrivateEditor.putString(KEY_MASTER_URI, masterAddress.getText().toString());
				prefsPrivateEditor.commit();
    			
				// Depending on whether the service is running, start or stop
    			if (!connectToggle.isChecked()) {
    				Log.i(logTag, "Starting background service.");
    				startService(intent);
    			} else {
    				Log.i(logTag, "Stopping background service.");
    				stopService(intent);
    			}
			}
    	});
        
        // Periodically update status of toggle button
        final Handler handler = new Handler();
        handler.postDelayed(new Runnable() {
			@Override
			public void run() {
				connectToggle.setChecked(AirboatService.isRunning);
				connectToggle.setEnabled(true);
				handler.postDelayed(this, 300);
			}
		}, 0);
        
        // Register handler for debug button
        final Button debugButton = (Button)findViewById(R.id.DebugButton);
        debugButton.setOnClickListener(new OnClickListener() {
			public void onClick(View v) {
				// Start up the debug control activity
				startActivity(new Intent(AirboatActivity.this, AirboatControlActivity.class));
				/* Broadcast Receive Test
				Intent i = new Intent(AirboatImpl.OBSTACLE);
				i.putExtra(OBSTACLE_DATA, true);
				
				Log.e("Osman", "Sent intent when click debug button");
				sendBroadcast(i);
				*/
				// start controller
			}
		});
        
        // Register handler for failsafe address that changes color 
		// if a valid hostname seems to be reached.
		// TODO: Move this to its own class!
		final AutoCompleteTextView failsafeAddress = (AutoCompleteTextView)findViewById(R.id.FailsafeAddress);
		failsafeAddress.addTextChangedListener(new TextWatcher() {
			
			final Handler handler = new Handler();
			final AtomicBoolean _isUpdating = new AtomicBoolean(false);
			final AtomicBoolean _isUpdated = new AtomicBoolean(false);
			
			final class TextUpdate extends AsyncTask<Void, Void, Integer> {
				private String hostname;
				
				@Override
				protected void onPreExecute() {
					hostname = failsafeAddress.getText().toString();
				}
				
				@Override
				protected Integer doInBackground(Void... urls) {
					int textBkgnd = 0xAAAA0000;
					
					_isUpdated.set(true);
					_isUpdating.set(true);
					
					try {
						// Try to open the host name in the text box, 
						// if it succeeds, change color accordingly
						if (hostname.trim().length() != 0 && InetAddress.getByName(hostname).isReachable(500))
				        	textBkgnd = 0xAA00AA00;
				    } catch (IOException e) {}
				    
				    return textBkgnd;
				}

				@Override
				protected void onPostExecute(Integer result) {
					failsafeAddress.setBackgroundColor(result);

					// Immediately reschedule if out of date, otherwise delay
				    if (!_isUpdated.get()) {
				    	new TextUpdate().execute((Void[])null);
				    } else {
				    	handler.postDelayed(new Runnable() {
							@Override
							public void run() {
								new TextUpdate().execute((Void[])null);
							}
						}, 2000);
				    }
				    
				    // In any case, we are now done updating
				    _isUpdating.set(false);
				}
			};
			
			@Override
			public void onTextChanged(CharSequence s, int start, int before, int count) {}
			
			@Override
			public void beforeTextChanged(CharSequence s, int start, int count, int after) {}
			
			@Override
			public void afterTextChanged(final Editable s) {
				
				_isUpdated.set(false);
				
				// If an update isn't already running, start one up
				if (!_isUpdating.get()) {
					new TextUpdate().execute((Void[])null);
				}
			}
		});
		
		// Register handler for failsafe toggle button
        final ToggleButton failsafeToggle = (ToggleButton)findViewById(R.id.FailsafeToggle);
        failsafeToggle.setOnClickListener(new OnClickListener() {
        	
			@Override
			public void onClick(View v) {
				// Don't allow re-clicking until the service status updates
				failsafeToggle.setEnabled(false);
				
				// Create an intent to properly start the vehicle server
    			Intent intent = new Intent(AirboatActivity.this, AirboatFailsafeService.class);
    			intent.putExtra(AirboatFailsafeIntent.HOSTNAME, failsafeAddress.getText().toString());
    			intent.putExtra(AirboatFailsafeIntent.HOME_POSE, new double[] {
    					_homePosition.pose.getX(),
    					_homePosition.pose.getY(),
    					_homePosition.pose.getZ()});
    			intent.putExtra(AirboatFailsafeIntent.HOME_ZONE, (byte)_homePosition.origin.zone);
    			intent.putExtra(AirboatFailsafeIntent.HOME_NORTH, _homePosition.origin.isNorth);
    			
				// Save the current BD addr and master URI
				SharedPreferences prefs = getSharedPreferences(PREFS_PRIVATE, Context.MODE_PRIVATE);
				Editor prefsPrivateEditor = prefs.edit();
				prefsPrivateEditor.putString(KEY_FAILSAFE_ADDR, failsafeAddress.getText().toString());
				prefsPrivateEditor.commit();
    			
    			if (!failsafeToggle.isChecked()) {
    				Log.i(logTag, "Starting failsafe service.");
    				startService(intent);
    			} else {
    				Log.i(logTag, "Stopping failsafe service.");
    				stopService(intent);
    			}
    		}
    	});
        
        // Periodically update status of failsafe button
        handler.postDelayed(new Runnable() {
			@Override
			public void run() {
				failsafeToggle.setChecked(AirboatFailsafeService.isRunning);
				failsafeToggle.setEnabled(AirboatService.isRunning);
				handler.postDelayed(this, 300);
			}
		}, 0);
        
        // Register handler for homing button
        final Button homeButton = (Button)findViewById(R.id.HomeButton);
        homeButton.setOnClickListener(new OnClickListener() {
			public void onClick(View v) {
				// Turn on GPS, get location, set as the home position
				final LocationManager locationManager = (LocationManager)getSystemService(Context.LOCATION_SERVICE);
				if (!locationManager.isProviderEnabled(LocationManager.GPS_PROVIDER)) {
					Toast.makeText(getApplicationContext(),
							"GPS must be turned on to set home location.",
							Toast.LENGTH_SHORT).show();
					return;
				}
				
				final AtomicBoolean gotHome = new AtomicBoolean(false); 
				final LocationListener ll = new LocationListener() {
					
					public void onStatusChanged(String provider, int status, Bundle extras) {}
					public void onProviderEnabled(String provider) {}
					public void onProviderDisabled(String provider) {}
					
					@Override
					public void onLocationChanged(Location location) {
						
						// Convert from lat/long to UTM coordinates
			        	UTM utmLoc = UTM.latLongToUtm(
			        				LatLong.valueOf(location.getLatitude(), location.getLongitude(), NonSI.DEGREE_ANGLE), 
			        				ReferenceEllipsoid.WGS84
			        			);
			        	_homePosition.pose = new Pose3D(
			        			utmLoc.eastingValue(SI.METER),
			        			utmLoc.northingValue(SI.METER),
			        			location.getAltitude(),
			        			0.0, 0.0, 0.0);
			        	_homePosition.origin = new Utm(utmLoc.longitudeZone(), utmLoc.latitudeZone() > 'O');
			        	AirboatFailsafeService.setHome(_homePosition);
						
			        	// Now that we have the GPS location, stop listening
			        	Toast.makeText(getApplicationContext(),
								"Home location set to: " + utmLoc,
								Toast.LENGTH_SHORT).show();
			        	Log.i(logTag, "Set home to " + utmLoc);
						locationManager.removeUpdates(this);
						gotHome.set(true);
					}
				};
        		locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 0, 0, ll);
        		
        		// Cancel GPS fix after a while
				final Handler _handler = new Handler();
				_handler.postDelayed(new Runnable() {
					
					@Override
					public void run() {
						if (!gotHome.get()) {
							// Cancel GPS lookup after 5 seconds
							locationManager.removeUpdates(ll);
							
							// Report failure to set home
				        	Toast.makeText(getApplicationContext(),
									"Home location not set: no GPS fix.",
									Toast.LENGTH_SHORT).show();
						}
					}
				}, 5000);
				
			}
		});
        
        // Set text boxes to previous values
        SharedPreferences prefs = getSharedPreferences(PREFS_PRIVATE, Context.MODE_PRIVATE);
        connectAddress.setText(prefs.getString(KEY_BT_ADDR, connectAddress.getText().toString()));
        masterAddress.setText(prefs.getString(KEY_MASTER_URI, masterAddress.getText().toString()));
        failsafeAddress.setText(prefs.getString(KEY_FAILSAFE_ADDR, failsafeAddress.getText().toString()));
    }
    
    @Override
    public void onResume() {
    	super.onResume();
    	
    	// Refresh current IP address
        final TextView addrText = (TextView)findViewById(R.id.IpAddressText);
        addrText.setText(getLocalIpAddress() + ":11411");
    }
    
    @Override
	public void onDestroy() {
    	
    	super.onDestroy();
    	unregisterReceiver(_amarinoReceiver);
    }
    
    /**
     * Helper function that retrieves first valid (non-loopback) IP address
     * over all available interfaces.
     * 
     * @return Text representation of current local IP address.
     */
	public String getLocalIpAddress() {
		try {
			for (Enumeration<NetworkInterface> en = NetworkInterface
					.getNetworkInterfaces(); en.hasMoreElements();) {
				NetworkInterface intf = en.nextElement();
				for (Enumeration<InetAddress> enumIpAddr = intf
						.getInetAddresses(); enumIpAddr.hasMoreElements();) {
					InetAddress inetAddress = enumIpAddr.nextElement();
					if (!inetAddress.isLoopbackAddress() && inetAddress.getAddress().length == 4) {
						return inetAddress.getHostAddress().toString();
					}
				}
			}
		} catch (SocketException ex) {
			Log.e(logTag, "Failed to get local IP.", ex);
		}
		return null;
	}

}
