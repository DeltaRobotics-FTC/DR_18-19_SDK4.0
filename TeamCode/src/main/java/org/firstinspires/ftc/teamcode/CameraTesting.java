package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import for_camera_opmodes.CameraPreview;
import for_camera_opmodes.LinearOpModeCamera;

@TeleOp (name = "Camera Testing", group = "")
public class CameraTesting extends LinearOpModeCamera
{
    Drive drive = new Drive();

    int mineralColorInt;

    @Override
    public void runOpMode()
    {

            if(isCameraAvailable())
            {
                setCameraDownsampling(1);

                startCamera();

                waitForStart();


                    if (imageReady()) {
                        int redValueLeft = -76800;
                        int blueValueLeft = -76800;
                        int greenValueLeft = -76800;

                        Bitmap rgbImage;
                        //The last value must correspond to the downsampling value from above
                        rgbImage = convertYuvImageToRgb(yuvImage, width, height, 1);


                        //This is for only saving the color image if needed.

                        for (int x = 380; x < 580; x++) {
                            for (int y = 280; y < 480; y++) {
                                if (x == 579 && y >= 479) {
                                    rgbImage.setPixel(x, y, Color.rgb(0, 255, 0));
                                }
                                if (x >= 0 && y == 450) {
                                    rgbImage.setPixel(x, y, Color.rgb(0, 255, 0));
                                }
                                if (x == 380 && y >= 450) {
                                    rgbImage.setPixel(x, y, Color.rgb(0, 255, 0));
                                }
                                if (x >= 380 && y == 479) {
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
                        /*telemetry.addData("redValueLeft", redValueLeft);
                        telemetry.addData("blueValueLeft", blueValueLeft);
                        telemetry.addData("greenValueLeft", greenValueLeft);
                        telemetry.addData("Width", rgbImage.getWidth());
                        telemetry.addData("Height", rgbImage.getHeight());
                        telemetry.update();*/


                        mineralColorInt = highestColor(redValueLeft, blueValueLeft, greenValueLeft);

                        telemetry.addData("Mineral", mineralColorInt);
                        if (mineralColorInt == 0) {
                            telemetry.addData("Mineral Color", "0 : Gold");
                        } else if (redValueLeft > 100 && blueValueLeft > 100 && greenValueLeft > 100) {
                            telemetry.addData("Mineral Color", "White");
                        } else {
                            telemetry.addData("Mineral Color", "Something's Wrong");
                        }
                        telemetry.update();
                        stopCamera();


                    }
                }

        }

}
