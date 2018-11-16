package for_camera_opmodes;

import android.app.Activity;
import android.content.Context;
import android.hardware.Camera;
import android.util.Log;
import android.view.Surface;
import android.view.SurfaceHolder;
import android.view.SurfaceView;

import java.io.IOException;


public class CameraPreview extends SurfaceView implements SurfaceHolder.Callback {
    private SurfaceHolder mHolder;
    private Camera mCamera;
    private static String TAG = "DEBUG";
    private Camera.PreviewCallback previewCallback;

    public CameraPreview(Context context, Camera camera, Camera.PreviewCallback previewCallback) {
        super(context);
        mCamera = camera;

        this.previewCallback = previewCallback;


        mHolder = getHolder();
        mHolder.addCallback(this);
        mHolder.setType(SurfaceHolder.SURFACE_TYPE_PUSH_BUFFERS);
    }

    public static void setCameraDisplayOrientation(Activity activity, int cameraId, Camera camera) {
        Camera.CameraInfo info = new Camera.CameraInfo();
        Camera.getCameraInfo(cameraId, info);
        int rotation = activity.getWindowManager().getDefaultDisplay().getRotation();
        int degrees = 0;
        switch (rotation) {
            case Surface.ROTATION_0:
                degrees = 0;
                break;
            case Surface.ROTATION_90:
                degrees = 90;
                break;
            case Surface.ROTATION_180:
                degrees = 180;
                break;
            case Surface.ROTATION_270:
                degrees = 270;
                break;
        }

        int result;
        if (info.facing == Camera.CameraInfo.CAMERA_FACING_FRONT) {
            result = (info.orientation + degrees) % 360;
            result = (360 - result) % 360;
        } else {  // back-facing
            result = (info.orientation - degrees + 360) % 360;
        }
        camera.setDisplayOrientation(result);
    }



    public void surfaceCreated(SurfaceHolder holder)
    {
        try {
            Camera.Parameters parameters = mCamera.getParameters();
            //         mCamera.setDisplayOrientation(90);
            setCameraDisplayOrientation(((Activity) this.getContext()), Camera.CameraInfo.CAMERA_FACING_FRONT, mCamera);

            mCamera.setParameters(parameters);

            mCamera.setPreviewCallback(previewCallback);
            mCamera.setPreviewDisplay(holder);
            mCamera.startPreview();
        } catch (IOException e) {
            Log.d(TAG, "Error setting camera preview: " + e.getMessage());
        }
    }

    public void surfaceDestroyed(SurfaceHolder holder) {
    }

    public void surfaceChanged(SurfaceHolder holder, int format, int w, int h) {

        if (mHolder.getSurface() == null){
            // preview surface does not exist
            return;
        }

        // stop preview before making changes
        try {
            mCamera.stopPreview();
        } catch (Exception e){
        }

        try {
            Camera.Parameters parameters = mCamera.getParameters();
            setCameraDisplayOrientation(((Activity) this.getContext()), Camera.CameraInfo.CAMERA_FACING_FRONT, mCamera);
            mCamera.setParameters(parameters);

            mCamera.setPreviewCallback(previewCallback);
            mCamera.setPreviewDisplay(mHolder);
            mCamera.startPreview();

        } catch (Exception e){
            Log.d(TAG, "Error starting camera preview: " + e.getMessage());
        }
    }
}