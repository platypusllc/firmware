package edu.cmu.ri.airboat.server;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.concurrent.atomic.AtomicMarkableReference;

import android.app.Activity;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.graphics.PixelFormat;
import android.hardware.Camera;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.Window;
import android.view.WindowManager;

/**
 * A dummy activity whose sole purpose is to take picture, save it to disk, 
 * and return the result to the calling function.
 * 
 * This class contains static helper methods used for taking picture and saving 
 * pictures to file and should generally not be instantiated directly.  
 * 
 * @author pkv
 *
 */
public class AirboatCameraActivity extends Activity implements SurfaceHolder.Callback {
	public static final String TAG = AirboatCameraActivity.class.getName();
	
	public static final String PICTURE_INTENT = "edu.cmu.ri.airboat.intent.action.PICTURE";
	public static final String QUALITY_EXTRA = "JpegQuality";
	public static final String WIDTH_EXTRA = "ImageWidth";
	public static final String HEIGHT_EXTRA = "ImageHeight";
	public static final String SAVE_EXTRA = "SaveImage";
	public static final String IMAGE_EXTRA = "ImageData";
	
	
	/**
	 * Takes a picture, blocking until the picture is complete, then returns
	 * the JPEG data for the picture.
	 * 
	 * This version of the method fixes picture size at 512x384.
	 *  
	 * @see AirboatCameraActivity#takePhoto(Context)
	 * @param context the context of the calling Activity or Service
	 * @return byte array containing a JPEG-encoded image, or zero-length array on failure
	 */
	public static synchronized byte[] takePhoto(final Context context) {
		return takePhoto(context, 512, 384);
	}
	
	/**
	 * Takes a picture, blocking until the picture is complete, then returns
	 * the JPEG data for the picture.
	 * 
	 * @param context the context of the calling Activity or Service
	 * @return byte array containing a JPEG-encoded image, or zero-length array on failure
	 */
	public static synchronized byte[] takePhoto(final Context context, int width, int height) {
		
		// Set up a structure to hold the JPEG data
		final AtomicMarkableReference<byte[]> dataRef = new AtomicMarkableReference<byte[]>(null, false);
		
		// Set up a receiver that will fill in the data structure
		IntentFilter pictureFilter = new IntentFilter();
		pictureFilter.addAction(PICTURE_INTENT);
		final BroadcastReceiver receiver = new BroadcastReceiver() {
			
			@Override
			public void onReceive(Context context, Intent intent) {

				// Load this image data into the receiver
				byte[] data = intent.getByteArrayExtra(IMAGE_EXTRA); 
				synchronized(dataRef) {
					dataRef.set(data, true);
					dataRef.notifyAll();
				}
				
				// Unregister this listener
				try { 
					context.unregisterReceiver(this);
				} catch (IllegalArgumentException e) {
					Log.w(TAG, "PictureReceiver failed to unregister.", e);
				}
			}
		};
		context.registerReceiver(receiver, pictureFilter);
		
		// Try to schedule a picture to be taken immediately	
		Intent intent = new Intent(context, AirboatCameraActivity.class);
		intent.setFlags(Intent.FLAG_ACTIVITY_NEW_TASK); // TODO: I don't know if this is the right flag here
		intent.putExtra(SAVE_EXTRA, true); // TODO: change this back at future point to disable always saving images
		intent.putExtra(QUALITY_EXTRA, 30);
		intent.putExtra(WIDTH_EXTRA, width);
		intent.putExtra(HEIGHT_EXTRA, height);
		context.startActivity(intent);
		
		// Wait for picture to be returned through atomic reference
		try {
			while (!dataRef.isMarked()) {
				synchronized(dataRef) {
					dataRef.wait();
				}
			}
			
			if (dataRef.getReference() != null) {
				return dataRef.getReference();
			} else {
				Log.w(TAG, "Camera activity returned null.");
				return new byte[0];
			}
		} catch (InterruptedException e) {
			Log.w(TAG, "Interrupted while taking photo", e);
			return new byte[0];
		}
	}
	
	/**
	 * Takes a picture and immediately saves it to disk.
	 * 
	 * @param context the context of the calling Activity or Service
	 */
	public static synchronized void savePhoto(final Context context) {

		// Try to schedule a picture to be taken immediately
		Intent intent = new Intent(context, AirboatCameraActivity.class);
		intent.setFlags(Intent.FLAG_ACTIVITY_NEW_TASK); // TODO: I don't know if this is the right flag here
		intent.putExtra(SAVE_EXTRA, true);
		context.startActivity(intent);
	}
	
	/**
	 * Asynchronous save operation that takes the result of the picture taking
	 * callback (a photo) and saves it to a date-stamped file.
	 */
	static class SavePhotoTask extends AsyncTask<byte[], String, String> {
		
		protected String doInBackground(byte[]... jpeg) {
			
			// Create a filename with the current date/time
			Date date = new Date();
			date.getDate();
			SimpleDateFormat sdf = new SimpleDateFormat ("yyyyMMdd_hhmmss");
            String filename = sdf.format(date) + ".jpg";
            
            // TODO: improve external file storage path handling (don't use Environment.getExternalStorageDirectory())
            // Make sure the photo path is ready to go (clear existing files)
			File photo = new File(Environment.getExternalStorageDirectory(), filename);
	        if (photo.exists()) photo.delete();
			
	        // Try to write the photo to the specified filename
			try {
				FileOutputStream fos = new FileOutputStream(photo.getPath());
				fos.write(jpeg[0]);
				fos.close();
			} catch (java.io.IOException e) {
				Log.e(TAG, "Exception in photoCallback", e);
			}
			
			return(null);
		}
	}
	
	private Camera _camera;
	private boolean _isPreviewRunning = false;
	private SurfaceView _surfaceView;
	private SurfaceHolder _surfaceHolder;
	private int _quality;
	private int _width;
	private int _height;
	private boolean _isSaved;
	
	/** Called when the activity is first created. */
	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);

		//getWindow().setFormat(PixelFormat.TRANSLUCENT);
		requestWindowFeature(Window.FEATURE_NO_TITLE);
		getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN,
				WindowManager.LayoutParams.FLAG_FULLSCREEN);
		setContentView(R.layout.camera);
		_surfaceView = (SurfaceView) findViewById(R.id.surface_camera);
		_surfaceHolder = _surfaceView.getHolder();
		_surfaceHolder.addCallback(this);
		_surfaceHolder.setType(SurfaceHolder.SURFACE_TYPE_PUSH_BUFFERS);
		
		_quality = getIntent().getIntExtra(QUALITY_EXTRA, 85);
		_width = getIntent().getIntExtra(WIDTH_EXTRA, 1600);
		_height = getIntent().getIntExtra(HEIGHT_EXTRA, 1200);
		_isSaved = getIntent().getBooleanExtra(SAVE_EXTRA, false);
	}
	
	@Override
	public void onPause() {
		super.onPause();

		// return empty image and exit
		final Intent intent = new Intent();
		intent.setAction(PICTURE_INTENT);
		intent.putExtra(IMAGE_EXTRA, new byte[0]);
		sendBroadcast(intent);
		setResult(RESULT_CANCELED, intent);
		finish();
	}
	
	public void surfaceCreated(SurfaceHolder holder) {
		try {
			// Attempt to open camera
			_camera = Camera.open();
			
			// Set desired camera parameters
			Camera.Parameters p = _camera.getParameters();
			p.setPictureFormat(PixelFormat.JPEG);
			p.setPictureSize(_width, _height);
			p.setJpegQuality(_quality);
			_camera.setParameters(p);
		} catch (RuntimeException e) {
			Log.w(TAG, "Unable to connect to camera.", e);

			// If opening the camera fails, return empty image and exit
			final Intent intent = new Intent();
			intent.setAction(PICTURE_INTENT);
			intent.putExtra(IMAGE_EXTRA, new byte[0]);
			sendBroadcast(intent);
			setResult(RESULT_CANCELED, intent);
			finish();
		}
	}
	
	public void surfaceChanged(SurfaceHolder holder, int format, int w, int h) {
		if (_camera == null) return;

		// stopPreview() will crash if preview is not running
		if (_isPreviewRunning) {
			_camera.stopPreview();
		}

		try {
			_camera.setPreviewDisplay(holder);
		} catch (IOException e) {
			Log.e(TAG, "Failed to set up preview", e);
		}
		_camera.startPreview();
		_isPreviewRunning = true;
		
		// Take the actual picture
		_camera.takePicture(null, null, mJpegPictureCallback);
	}

	public void surfaceDestroyed(SurfaceHolder holder) {
		if (_camera != null) {
			_camera.stopPreview();
			_camera.release();
		}
		
		_isPreviewRunning = false;	
	}
		
	Camera.PictureCallback mJpegPictureCallback = new Camera.PictureCallback() {
		public void onPictureTaken(byte[] data, Camera c) {
			if (data == null) {
				Log.e(TAG, "Failed to get valid image.");
			}

			// Spawn off background process to save image
			if (_isSaved) {
				new SavePhotoTask().execute(data);
			}

			// Return image result as new intent
			final Intent intent = new Intent();
			intent.setAction(PICTURE_INTENT);
			intent.putExtra(IMAGE_EXTRA, data);
			sendBroadcast(intent);
			setResult(RESULT_OK, intent);
			finish();
		}
	};
}
