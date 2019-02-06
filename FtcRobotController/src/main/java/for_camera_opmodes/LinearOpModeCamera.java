package for_camera_opmodes;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.ImageFormat;
import android.graphics.YuvImage;
import android.hardware.Camera;
import android.os.Environment;
import android.util.Log;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import java.io.File;
import java.io.FileOutputStream;
import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */
public class LinearOpModeCamera extends LinearOpMode
{

    public Camera camera;
    public CameraPreview preview;

    public int width;
    public int height;
    public YuvImage yuvImage = null;

    volatile private boolean imageReady = false;

    private int looped = 0;
    private String data;
    private int ds = 1; // downsampling parameter

    @Override
    // should be overwritten by extension class
    public void runOpMode() throws InterruptedException {

    }


    public Camera.PreviewCallback previewCallback = new Camera.PreviewCallback() {
        public void onPreviewFrame(byte[] data, Camera camera) {
            try {
                Camera.Parameters parameters = camera.getParameters();
                width = parameters.getPreviewSize().width;
                height = parameters.getPreviewSize().height;
                yuvImage = new YuvImage(data, ImageFormat.NV21, width, height, null);
                imageReady = true;
                looped += 1;
            } catch (Exception e) {

            }
        }
    };

    public void setCameraDownsampling(int downSampling) {
        ds = downSampling;
    }

    public boolean imageReady() {
        return imageReady;
    }

    public boolean isCameraAvailable() {
        int cameraId = -1;
        Camera cam = null;
        int numberOfCameras = Camera.getNumberOfCameras();
        for (int i = 0; i < numberOfCameras; i++) {
            Camera.CameraInfo info = new Camera.CameraInfo();
            Camera.getCameraInfo(i, info);
            if (info.facing == Camera.CameraInfo.CAMERA_FACING_FRONT) { // Camera.CameraInfo.CAMERA_FACING_FRONT or BACK
                cameraId = i;
                break;
            }
        }
        try {
            cam = Camera.open(cameraId);
        } catch (Exception e) {
            Log.e("Error", "Camera Not Available!");
            return false;
        }
        cam.release();
        cam = null;
        return true;
    }

    public Camera openCamera(int cameraInfoType) {
        int cameraId = -1;
        Camera cam = null;
        int numberOfCameras = Camera.getNumberOfCameras();
        for (int i = 0; i < numberOfCameras; i++) {
            Camera.CameraInfo info = new Camera.CameraInfo();
            Camera.getCameraInfo(i, info);
            if (info.facing == cameraInfoType) { // Camera.CameraInfo.CAMERA_FACING_FRONT or BACK
                cameraId = i;
                break;
            }
        }
        try {
            cam = Camera.open(cameraId);
        } catch (Exception e) {
            Log.e("Error", "Can't Open Camera");
        }
        return cam;
    }

    public void startCamera() {

        camera = openCamera(Camera.CameraInfo.CAMERA_FACING_FRONT);
        camera.setPreviewCallback(previewCallback);

        Camera.Parameters parameters = camera.getParameters();

        width = parameters.getPreviewSize().width / ds;
        height = parameters.getPreviewSize().height / ds;
        parameters.setPreviewSize(width, height);

        camera.setParameters(parameters);

        data = parameters.flatten();

        if (preview == null) {
           ((FtcRobotControllerActivity) hardwareMap.appContext).initPreviewLinear(camera, this, previewCallback);
        }
    }

    public void stopCameraInSecs(int duration) {
        Thread cameraKillThread = new Thread(new CameraKillThread(duration));

        cameraKillThread.start();
    }

    public class CameraKillThread implements Runnable {
        int dur;

        public CameraKillThread(int duration) {
            dur = duration;
        }

        public void run() {
            try {
                Thread.sleep(dur * 1000, 0);
            } catch (InterruptedException ex) {

            }

            stopCamera();
            imageReady = false;
        }
    }

    public void stopCamera() {
        if (camera != null) {
            if (preview != null) {
                ((FtcRobotControllerActivity) hardwareMap.appContext).removePreviewLinear(this);
                preview = null;
            }
            camera.stopPreview();
            camera.setPreviewCallback(null);
            if (camera != null) {
                camera.release();
            }
            camera = null;
            imageReady = false;
        }
    }

    //Below this point does not change camera functionality//

    static public int red(int pixel) {
        return (pixel >> 16) & 0xff;
    }

    static public int green(int pixel) {
        return (pixel >> 8) & 0xff;
    }

    static public int blue(int pixel) {
        return pixel & 0xff;
    }

    static public int gray(int pixel) {
        return (red(pixel) + green(pixel) + blue(pixel));
    }

    static public int highestColor(int red, int green, int blue) {
        int[] color = {red, green, blue};
        int value = 0;
        for (int i = 1; i < 3; i++) {
            if (color[value] < color[i]) {
                value = i;
            }
        }
        return value;
    }

    // returns ROTATED image, to match preview window
    static public Bitmap convertYuvImageToRgb(YuvImage yuvImage, int width, int height, int downSample) {
        return OpModeCamera.convertYuvImageToRgb(yuvImage, width, height, downSample);
    }

    public int normalizePixels(int value) {
        value /= 100000;
        return value;
    }

    public void SaveImage(Bitmap finalBitmap) {

        FileOutputStream out;
        String root = Environment.getExternalStorageDirectory().toString();
        File myDir = new File(root);
        myDir.mkdirs();
        SimpleDateFormat s = new SimpleDateFormat("ddhhmmss");
        String format = s.format(new Date());
        String fname = format +".jpg";
        File file = new File (myDir, fname);
        if (file.exists ()) file.delete ();
        try {
            out = new FileOutputStream(file);
            finalBitmap.compress(Bitmap.CompressFormat.JPEG, 90, out);
            out.flush();
            out.close();

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void SaveRGB (Bitmap colorBitmap)
    {
        String pixelColorMap = "";
        String rtemp;
        String gtemp;
        String btemp;
        String xv;
        String yv;
        int pixelPlaceholder;
        int Rtemp;
        int Gtemp;
        int Btemp;
        for(int y = 0; y < 640; y++)
        {
            for(int x = 0; x < 480; x++)
            {
                pixelPlaceholder = colorBitmap.getPixel(x,y);
                xv = Integer.toString(x);
                yv = Integer.toString(y);
                Rtemp = red(pixelPlaceholder);
                rtemp = Integer.toString(Rtemp);
                Gtemp = green(pixelPlaceholder);
                gtemp = Integer.toString(Gtemp);
                Btemp = blue(pixelPlaceholder);
                btemp = Integer.toString(Btemp);

                pixelColorMap = pixelColorMap + x + "," + y + " - ";
                pixelColorMap = pixelColorMap + rtemp + ",";
                pixelColorMap = pixelColorMap + gtemp + ",";
                pixelColorMap = pixelColorMap + btemp + System.getProperty("line.separator");
            }
        }
    }


    //Below Here is Game Specific Methods for 2017-18//\



    public void SaveJewelColorLog (int[][] colorValues)
    {
        String text = "";
        String root = Environment.getExternalStorageDirectory().toString();
        File myDir = new File(root);
        myDir.mkdirs();
        SimpleDateFormat s = new SimpleDateFormat("ddhhmmss");
        String format = s.format(new Date());
        String fname = "ColorData" + format +".txt";
        File file = new File (myDir, fname);
        if (file.exists ()) file.delete ();
        try {
            FileOutputStream out = new FileOutputStream(file);

            for(int x = 300; x < 500; x++)
            {
                text = text +  x + "->"  + "R" + colorValues[x][0] + "  G" + colorValues[x][2] + "  B" + colorValues[x][1] + " - " + colorValues[x][3] + "\n";
            }
            out.write(text.getBytes());

            out.flush();
            out.close();

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void SaveCryptoboxColorLog (int[][] colorValues, int width)
    {
        String text = "";
        String root = Environment.getExternalStorageDirectory().toString();
        File myDir = new File(root);
        myDir.mkdirs();
        SimpleDateFormat s = new SimpleDateFormat("ddhhmmss");
        String format = s.format(new Date());
        String fname = "CryptoboxData " + format +".txt";
        File file = new File (myDir, fname);
        if (file.exists ()) file.delete ();
        try {
            FileOutputStream out = new FileOutputStream(file);

            for(int x = 0; x < width; x++)
            {
                text = text + "Line " + x + ":  " + colorValues[x][0] + " " + colorValues[x][1] + " " + colorValues[x][2] + "         "
                        + colorValues[x][4] + " " + colorValues[x][5] + " " + colorValues[x][6] + "          " + colorValues[x][8] + " " + colorValues[x][9] + " " + colorValues[x][10] + '\n';
            }
            out.write(text.getBytes());

            out.flush();
            out.close();

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public int[] FindJewelsCenter (Bitmap cameraBitmap)
    {
        int[][] colorValues = new int[960][4];
        int[] returnValues = new int[960];
        int width1 = cameraBitmap.getWidth();
        int lineRightX = -1;
        int tempPixel;
        int white = 140; //To be used with greater than
        int black = 100; //To be used with less than
        int x = width1 - 1;

        //Camera field of view should equal 960x1280 as long as downsampling is equal to 1
            for(x = 0; x < width1 - 1; x++)
            {
                tempPixel = cameraBitmap.getPixel(x,(850));
                colorValues[x][0] = red(tempPixel);
                colorValues[x][1] = green(tempPixel);
                colorValues[x][2] = blue(tempPixel);
                if(red(tempPixel) > white && blue(tempPixel) > white && green(tempPixel) > white)
                {
                    colorValues[x][3] = 0;
                    returnValues[x] = 0;
                }
                else if(red(tempPixel) < black && blue(tempPixel) < black && green(tempPixel) < black)
                {
                    colorValues[x][3] = 1;
                    returnValues[x] = 1;
                }
                else if((red(tempPixel) > 120) && (green(tempPixel) < 120) && (green(tempPixel) > 70) && (blue(tempPixel) < 90))
                {
                    colorValues[x][3] = 2;
                    returnValues[x] = 2;
                }
            }
        SaveJewelColorLog(colorValues);
        return returnValues;
    }

    public int[] FindCryptoboxSides (Bitmap cameraBitmap, String allianceColor)
    {
        int cryptoboxWidth = cameraBitmap.getWidth();
        int cryptoboxHeight = cameraBitmap.getHeight();
        int[] sideColor = new int[3];
        int[][] colorValues = new int[width][12];
        int[] resultingMovement = new int[5];
        int tempPixel1;
        int tempPixel2;
        int tempPixel3;
        int y1 = cryptoboxHeight - (int)(cryptoboxHeight *.25);
        //y1 is located 1/4 of the way down the screen
        int y2 = cryptoboxHeight - (int)(cryptoboxHeight *.5);
        //y2 is located 1/2 of the way down the screen
        int y3 = cryptoboxHeight - (int)(cryptoboxHeight *.75);
        //y3 is located 3/4 of the way down the screen

        if(allianceColor.equals("RED"))
        {
            sideColor[0] = 255;
            //Red Value
            sideColor[1] = 50;
            //Green Value
            sideColor[2] = 50;
            //Blue Value
        }
        if(allianceColor.equals("BLUE"))
        {
            sideColor[0] = 50;
            //Red Value
            sideColor[1] = 50;
            //Green Value
            sideColor[2] = 255;
            //Blue Value
        }

        for(int x = 0; x < cryptoboxWidth; x++)
        {
            tempPixel1 = cameraBitmap.getPixel(x, y1);
            tempPixel2 = cameraBitmap.getPixel(x, y2);
            tempPixel3 = cameraBitmap.getPixel(x, y3);
            colorValues[x][0] = red(tempPixel1);
            colorValues[x][1] = green(tempPixel1);
            colorValues[x][2] = blue(tempPixel1);
            colorValues[x][4] = red(tempPixel2);
            colorValues[x][5] = green(tempPixel2);
            colorValues[x][6] = blue(tempPixel2);
            colorValues[x][8] = red(tempPixel3);
            colorValues[x][9] = green(tempPixel3);
            colorValues[x][10] = blue(tempPixel3);
        }
        resultingMovement[0] = cryptoboxHeight - (int)(cryptoboxHeight *.25);
        resultingMovement[1] = cryptoboxHeight - (int)(cryptoboxHeight *.5);
        resultingMovement[2] = cryptoboxHeight - (int)(cryptoboxHeight *.75);
        SaveCryptoboxColorLog(colorValues, cryptoboxWidth);

        return resultingMovement;
    }

}