package edu.cmu.ri.airboat.server;

import java.text.DecimalFormat;
import java.util.Arrays;

import android.app.Activity;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Handler;
import android.os.IBinder;
import android.util.Log;
import android.view.KeyEvent;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.inputmethod.EditorInfo;
import android.widget.CheckBox;
import android.widget.EditText;
import android.widget.SeekBar;
import android.widget.SeekBar.OnSeekBarChangeListener;
import android.widget.TextView;
import android.widget.TextView.OnEditorActionListener;

import com.google.code.microlog4android.LoggerFactory;

import edu.cmu.ri.crw.VehicleServer;
import edu.cmu.ri.crw.data.Twist;

/**
 * Android activity to display the debug screen to modify server parameters using a GUI
 * 
 * @author pkv
 * @author kshaurya
 *
 */
public class AirboatControlActivity extends Activity {
	private static final com.google.code.microlog4android.Logger logger = LoggerFactory.getLogger();
	private static final String logTag = AirboatControlActivity.class.getName();
	
	// Update rates for PID, velocity and status
	public static final long VEL_UPDATE_MS = 400;
	public static final long PID_UPDATE_MS = 1000;
	
	// Ranges for thrust and rudder signals
	public static final double THRUST_MIN =  0.0;
	public static final double THRUST_MAX =  1.0;
	public static final double RUDDER_MIN =  1.0; // Reversed to match +Z rotation.
	public static final double RUDDER_MAX = -1.0;
	
	// Contains a reference to the airboat service, or null if service is not running 
	private AirboatService _airboatService = null;
	
	// Indicates if we have a valid reference to the airboat service.
	private boolean _isBound = false;
	
	// Stores current velocity values.
	private Twist _velocities = new Twist(AirboatImpl.DEFAULT_TWIST);
	
	// Timing functions to regularly update GUI
	private Handler _velHandler = null;
	private Runnable _velCallback = null;
	
	private Handler _pidHandler = null;
	private Runnable _pidCallback = null;
	
	private AsyncTaskFactory<Void, Void, double[][]> _updatePid = null;
	
	// Create a factory interface that generates the necessary TimerTasks
	interface AsyncTaskFactory<A, B, C> {
		public AsyncTask<A, B, C> instance();
	}
	
	// Converts from progress bar value to linear scaling between min and max
	private double fromProgressToRange(int progress, double min, double max) {
		return (min + (max - min) * (progress)/100.0);
	}
	
	/** Sets PID gains if the thrust text boxes are edited */
	private class PidKeyListener implements OnEditorActionListener {
		private final int _axis;
		private final EditText _pText, _iText, _dText;
		
		public PidKeyListener(int axis, EditText pText, EditText iText, EditText dText) {
			_axis = axis;
			_pText = pText;
			_iText = iText;
			_dText = dText;
		}
		
		public boolean onEditorAction(TextView v, int actionId, KeyEvent event) {

			// Ignore key presses other than "Done"
			if (actionId != EditorInfo.IME_ACTION_DONE)
				return false;
			
			// Don't do anything if we can't get access to the controller
			if ((_airboatService == null) || (_airboatService.getServer()== null)) 
				return false;
			
			// Set the PID gains using the values from the text boxes
			try {
				double kp = Double.parseDouble(_pText.getText().toString());
				double ki = Double.parseDouble(_iText.getText().toString());
				double kd = Double.parseDouble(_dText.getText().toString());
				_airboatService.getServer().setGains(_axis, new double[] {kp, ki, kd});
			} catch (NumberFormatException ex) {
				Log.w(logTag, "Failed to parse gain.", ex);
			}
			
			return false;
		}
	}
	
	/** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {

    	// Create the "control" layout from the included XML file 
    	super.onCreate(savedInstanceState);
        setContentView(R.layout.control);

        // Get references to all the GUI elements
        final CheckBox connectedBox = (CheckBox)findViewById(R.id.ConnectedBox);
        final CheckBox autonomousBox = (CheckBox)findViewById(R.id.AutonomousBox);
        
        final TextView rudderValue = (TextView)findViewById(R.id.RudderValue);
        final SeekBar rudderSlider = (SeekBar)findViewById(R.id.RudderSlider);
        final EditText rudderPGain = (EditText)findViewById(R.id.RudderPGain);
        final EditText rudderIGain = (EditText)findViewById(R.id.RudderIGain);
        final EditText rudderDGain = (EditText)findViewById(R.id.RudderDGain);
        
        final TextView thrustValue = (TextView)findViewById(R.id.ThrustValue);
        final SeekBar thrustSlider = (SeekBar)findViewById(R.id.ThrustSlider);
        final EditText thrustPGain = (EditText)findViewById(R.id.ThrustPGain);
        final EditText thrustIGain = (EditText)findViewById(R.id.ThrustIGain);
        final EditText thrustDGain = (EditText)findViewById(R.id.ThrustDGain);
        
		// If the PID values for thrust are changed, send them to the server
        PidKeyListener thrustPidListener = new PidKeyListener(0, thrustPGain, thrustIGain, thrustDGain);
        thrustPGain.setOnEditorActionListener(thrustPidListener);
        thrustIGain.setOnEditorActionListener(thrustPidListener);
        thrustDGain.setOnEditorActionListener(thrustPidListener);
        
        // If the PID values for rudder are changed, send them to the server
        PidKeyListener rudderPidListener = new PidKeyListener(5, rudderPGain, rudderIGain, rudderDGain);
        rudderPGain.setOnEditorActionListener(rudderPidListener);
        rudderIGain.setOnEditorActionListener(rudderPidListener);
        rudderDGain.setOnEditorActionListener(rudderPidListener);
        
        // If the Autonomous check box is pressed, send a command to change its state
        autonomousBox.setOnClickListener(new OnClickListener() {
			public void onClick(View v) {
				// Don't do anything if we can't get access to the controller
				if ((_airboatService == null) || (_airboatService.getServer() == null)) 
					return;
				
				// Stop navigation
				_airboatService.getServer().setAutonomous(!_airboatService.getServer().isAutonomous());
			}
		});
        
        // When the rudder slider is moved, send a command to change it
        rudderSlider.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {
			
			public void onStopTrackingTouch(SeekBar seekBar) {}
			public void onStartTrackingTouch(SeekBar seekBar) {}
			
			public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
				// Don't do anything if we can't get access to the controller
				if ((_airboatService == null) || (_airboatService.getServer() == null)) 
					return;
				
				// Ignore our own update events
				if (!fromUser)
					return;
				
				// Send a new velocity command
				Twist twist = _airboatService.getServer().getVelocity();
				double yaw = fromProgressToRange(rudderSlider.getProgress(), RUDDER_MIN, RUDDER_MAX);
				twist.drz(yaw);
				_airboatService.getServer().setVelocity(twist);
			}
		});
        
        // When the thrust slider is moved, send a command to change it
        thrustSlider.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {
			
			public void onStopTrackingTouch(SeekBar seekBar) {}
			public void onStartTrackingTouch(SeekBar seekBar) {}
			
			public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
				// Don't do anything if we can't get access to the controller
				if ((_airboatService == null) || (_airboatService.getServer() == null)) 
					return;

				// Ignore our own update events
				if (!fromUser)
					return;
				
				// Send a new velocity command
				Twist twist = _airboatService.getServer().getVelocity();
				double thrust = fromProgressToRange(thrustSlider.getProgress(), THRUST_MIN, THRUST_MAX);
				twist.dx(thrust);
				_airboatService.getServer().setVelocity(twist);
			}
		});
        
        // Schedule a regular update of the velocity and status functions
        _velHandler = new Handler();
        _velCallback = new Runnable() {
        	// Initialize a formatter to display velocity strings in a reasonable format
			DecimalFormat velFormatter = new DecimalFormat("####.###");

			public void run() {				
				
				// Don't do anything if we can't get access to the controller
				if ((_airboatService == null) || (_airboatService.getServer() == null))
					return;
					
				// Update the autonomous and connected values
				connectedBox.setChecked(_airboatService.getServer().isConnected());
				autonomousBox.setChecked(_airboatService.getServer().isAutonomous());
				
				// Update the velocities
				_velocities = _airboatService.getServer().getVelocity();

				thrustValue.setText(velFormatter.format(_velocities.dx() * 100.0) + " %");
				thrustValue.invalidate();
	
				rudderValue.setText(velFormatter.format(_velocities.drz() * 100.0) + " %");
				rudderValue.invalidate();
	
				// Reschedule the next iteration of this update
				_velHandler.postDelayed(_velCallback, VEL_UPDATE_MS);
			}
		};
    	
		
    	// Schedule a regular update of the PID functions
		_pidHandler = new Handler();
		_pidCallback = new Runnable() {
			public void run() {
				_updatePid.instance().execute((Void)null);
			}
		};
		_updatePid =  new AsyncTaskFactory<Void, Void, double[][]>() {
			public AsyncTask<Void, Void, double[][]> instance() {
				return new AsyncTask<Void, Void, double[][]>() {
					@Override
					protected double[][] doInBackground(Void... params) {
						
						// Don't do anything if we can't get access to the controller
						VehicleServer server;
						if ((_airboatService == null) || ((server = _airboatService.getServer()) == null))
							return new double[][] { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };
		
						// Update the PID gains
						// UPDATE: use pidRudder thrust from server code rather than arduino
						double[] pidThrust = server.getGains(0);
						// fix below statement; causes server to crash when shut down
						double[] pidRudder = server.getGains(5);
						
						return new double[][] { pidThrust, pidRudder };
					}
					
					@Override
					protected void onPostExecute(double[][] result) {
						if (!thrustPGain.hasFocus()) {
							thrustPGain.setText(Double.toString(result[0][0]));
							thrustPGain.invalidate();
						}
						if (!thrustIGain.hasFocus()) {
							thrustIGain.setText(Double.toString(result[0][1]));
							thrustIGain.invalidate();
						}
						if (!thrustDGain.hasFocus()) {
							thrustDGain.setText(Double.toString(result[0][2]));
							thrustDGain.invalidate();
						}
						
						if (!rudderPGain.hasFocus()) {
							rudderPGain.setText(Double.toString(result[1][0]));
							rudderPGain.invalidate();
						}
						if (!rudderIGain.hasFocus()) {
							rudderIGain.setText(Double.toString(result[1][1]));
							rudderIGain.invalidate();
						}
						if (!rudderDGain.hasFocus()) {
							rudderDGain.setText(Double.toString(result[1][2]));
							rudderDGain.invalidate();
						}
	
						logger.info("PID: " + "0 " + Arrays.toString(result[0]));
						logger.info("PID: " + "5 " + Arrays.toString(result[1]));
						
						// Reschedule the next iteration of this update
						_pidHandler.postDelayed(_pidCallback, PID_UPDATE_MS);
					}
				};
			}
		};
	}

    /** 
     * Listener that handles changes in connections to the airboat service 
     */ 
    private ServiceConnection _connection = new ServiceConnection() {
        public void onServiceConnected(ComponentName className, IBinder service) {
            // This is called when the connection with the service has been
            // established, giving us the service object we can use to
            // interact with the service.
            _airboatService = ((AirboatService.AirboatBinder)service).getService();
        }

        public void onServiceDisconnected(ComponentName className) {
            // This is called when the connection with the service has been
            // unexpectedly disconnected -- that is, its process crashed.
            _airboatService = null;
        }
    };

    void doBindService() {
        // Establish a connection with the service.  We use an explicit
        // class name because we want a specific service implementation.
        if (!_isBound) {
        	bindService(new Intent(this, AirboatService.class), _connection, Context.BIND_AUTO_CREATE);
        	_isBound = true;
        }
    }

    void doUnbindService() {
        // Detach our existing connection.
    	if (_isBound) {
            unbindService(_connection);
            _isBound = false;
        }
    }
    
    @Override
    protected void onStart() {
    	super.onStart();
    	doBindService();
    	
    	_velHandler.removeCallbacks(_velCallback);
        _velHandler.post(_velCallback);
        
        _pidHandler.removeCallbacks(_pidCallback);
        _pidHandler.post(_pidCallback);
    }

    @Override
    protected void onStop() {
    	super.onStop();
        doUnbindService();
       
        _velHandler.removeCallbacks(_velCallback);
        _pidHandler.removeCallbacks(_pidCallback);
    }
}
