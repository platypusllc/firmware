/**
 * 
 */
package com.platypus.android.server.module;

import java.io.IOException;
import java.util.List;

import android.graphics.ImageFormat;
import android.hardware.Camera;
import android.hardware.Camera.PreviewCallback;
import android.hardware.Camera.Size;
import android.util.JsonReader;
import android.util.Log;
import android.view.SurfaceView;

import com.platypus.android.server.VehicleModule;
import com.platypus.android.server.VehicleService;
import com.platypus.protobuf.Image;
import com.platypus.protobuf.PlatypusResponse;
import com.squareup.wire.ByteString;

/**
 * Connects to the phone camera and handles configuration and streaming of 
 * camera imagery.
 * 
 * @author pkv
 *
 */
// TODO: Add mechanism to set framerate using manual addPreviewCallback with TimerTask
public class CameraModule implements VehicleModule {
    private static String TAG = CameraModule.class.getName();

    private VehicleService mService;
    private Camera mCamera;
    //private MediaRecorder mMediaRecorder;
    private SurfaceView mSurfaceView;
    
    /* (non-Javadoc)
     * @see com.platypus.android.server.VehicleModule#start(com.platypus.android.server.VehicleService)
     */
    public void start(VehicleService vehicle) {

        // Open camera and create offscreen surface view for preview
        mCamera = Camera.open();
        mSurfaceView = new SurfaceView(mService);
        
        // Set up camera parameters as needed
        Camera.Parameters params = mCamera.getParameters();
        {
            params.setJpegQuality(70);
        }
        mCamera.setParameters(params);
        
        // Set up preview window to render to offscreen surface
        params = mCamera.getParameters();
        {
            final List<Size> listSize = params.getSupportedPreviewSizes();
            Size mPreviewSize = listSize.get(2); // TODO: what the heck size is this?
            params.setPreviewSize(mPreviewSize.width, mPreviewSize.height);
            params.setPreviewFormat(ImageFormat.JPEG);
        }
        mCamera.setParameters(params);

        // Start rendering preview to offscreen surface
        try {
            mCamera.setPreviewDisplay(mSurfaceView.getHolder());
            mCamera.setPreviewCallback(mPreviewCallback);
            mCamera.startPreview();
        } catch (IOException e) {
            Log.e(TAG, "Failed to start camera preview.", e);
        }

        // We don't need to control the camera specifically anymore
        // TODO: is this actually true?
        mCamera.unlock();

        /*
        mMediaRecorder = new MediaRecorder();
        mMediaRecorder.setCamera(mCamera);
        mMediaRecorder.setAudioSource(MediaRecorder.AudioSource.MIC);
        mMediaRecorder.setVideoSource(MediaRecorder.VideoSource.CAMERA);
        mMediaRecorder.setOutputFormat(MediaRecorder.OutputFormat.MPEG_4);
        mMediaRecorder.setAudioEncoder(MediaRecorder.AudioEncoder.DEFAULT);
        mMediaRecorder.setVideoEncoder(MediaRecorder.VideoEncoder.DEFAULT);
        mMediaRecorder.setOutputFile("/sdcard/filenamevideo.mp4");
        mMediaRecorder.setVideoFrameRate(30);
        mMediaRecorder.setVideoSize(mPreviewSize.width, mPreviewSize.height);
        mMediaRecorder.setPreviewDisplay(mSurfaceHolder.getSurface());

        mMediaRecorder.prepare();
        mMediaRecorder.start();
        */ 
    }

    /* (non-Javadoc)
     * @see com.platypus.android.server.VehicleModule#stop()
     */
    public void stop() {

        /*
        mMediaRecorder.stop();
        mMediaRecorder.reset();
        mMediaRecorder.release();
        */

        // Shutdown the camera and preview
        try {
            mCamera.reconnect();
            mCamera.stopPreview();
            mCamera.release();
        } catch (IOException e) {
            Log.e(TAG, "Failed to disconnect from camera.", e);
        }
    }

    private PreviewCallback mPreviewCallback = new PreviewCallback() {
        public void onPreviewFrame(byte[] data, Camera camera) {
            
            // Get size of preview image
            // TODO: cache size of preview image?  Need to profile this.
            Size imageSize = camera.getParameters().getPreviewSize();
            
            // Construct a response structure containing image
            PlatypusResponse response = new PlatypusResponse.Builder()
                    .image(new Image.Builder()
                            .data(ByteString.of(data))
                            .width(imageSize.width)
                            .height(imageSize.height).build())
                    .build();
            
            // Send the image to service
            // TODO: send this to MADARA
        }
    };
    
    /* (non-Javadoc)
     * @see com.platypus.android.server.VehicleModule#onAccessoryReceive(android.util.JsonReader)
     */
    public boolean onAccessoryReceive(String name, JsonReader response) {
        // This module does not interact with the accessory.
        return false;
    }
}
