package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import for_camera_opmodes.LinearOpModeCamera;

public class CameraTesting extends LinearOpModeCamera
{
    RobotHardware robot = new RobotHardware();
    Drive drive = new Drive();

    int mineralColorInt;

    @Override
    public void runOpMode()
    {
        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive())
        {
            if(isCameraAvailable())
            {
                setCameraDownsampling(1);

                startCamera();

                if(imageReady())
                {
                    int redValueLeft = -76800;
                    int blueValueLeft = -76800;
                    int greenValueLeft = -76800;

                    Bitmap rgbImage;
                    //The last value must correspond to the downsampling value from above
                    rgbImage = convertYuvImageToRgb(yuvImage, width, height, 1);

                    telemetry.addData("Width", rgbImage.getWidth());
                    telemetry.addData("Height", rgbImage.getHeight());
                    telemetry.update();

                    //This is for only saving the color image if needed.

                    for (int x = 480; x < 680; x++)
                    {
                        for (int y = 850; y < 1280; y++)
                        {
                            if (x == 679 && y >= 850)
                            {
                                rgbImage.setPixel(x, y, Color.rgb(0, 255, 0));
                            }
                            if (x >= 0 && y == 850)
                            {
                                rgbImage.setPixel(x, y, Color.rgb(0, 255, 0));
                            }
                            if (x == 480 && y >= 850)
                            {
                                rgbImage.setPixel(x, y, Color.rgb(0, 255, 0));
                            }
                            if (x >= 480 && y == 1279)
                            {
                                rgbImage.setPixel(x, y, Color.rgb(0, 255, 0));
                            }
                        }
                    }

                    SaveImage(rgbImage);

                    //Analyzing Jewel Color
                    for (int x = 550; x < 850; x++) {
                        for (int y = 900; y < 1280; y++) {
                            int pixel = rgbImage.getPixel(x, y);
                            redValueLeft += red(pixel);
                            blueValueLeft += blue(pixel);
                            greenValueLeft += green(pixel);
                        }
                    }
                    redValueLeft = normalizePixels(redValueLeft);
                    blueValueLeft = normalizePixels(blueValueLeft);
                    greenValueLeft = normalizePixels(greenValueLeft);
                    telemetry.addData("redValueLeft", redValueLeft);
                    telemetry.addData("blueValueLeft", blueValueLeft);
                    telemetry.addData("greenValueLeft", greenValueLeft);

                   mineralColorInt = highestColor(redValueLeft, blueValueLeft, greenValueLeft);

                    telemetry.addData("Minearl", mineralColorInt);
                    if (mineralColorInt == 0)
                    {
                        telemetry.addData("Mineral Color", "0 : Gold");
                    } else if (redValueLeft > 100 && blueValueLeft > 100 && greenValueLeft > 100)
                    {
                        telemetry.addData("Mineral Color", "White");
                    }
                    else {
                        telemetry.addData("Mineral Color", "Something's Wrong");
                    }
                    telemetry.update();

                    stopCamera();
                }
            }
        }
    }
}
