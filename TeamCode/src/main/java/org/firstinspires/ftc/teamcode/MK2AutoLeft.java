package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.media.tv.TvInputManager;
import android.transition.Slide;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import for_camera_opmodes.LinearOpModeCamera;

/**
 * Created by User on 4/19/2018.
 */


@Autonomous(name="MK2AutoLeft",group = "")
//@Disabled
public class MK2AutoLeft extends LinearOpModeCamera {
    BNO055IMU imu;
    Orientation angles;

    RobotHardware_MK2 robot = new RobotHardware_MK2();
    Drive_MK2 drive = new Drive_MK2();

    final int SLEEP_TIME = 500;

    int mineralColorInt;


    //Min and max values for the bounds of the area of the image that will be analyzed
    int xMin = 250;
    int xMax = 625;

    int yMin = 925;
    int yMax = 1175;

    MineralPositions mineralPositions = MineralPositions.LEFT;

    public void runOpMode() {
        robot.init(hardwareMap);
        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters(); //Declares parameters object forIMU
        parametersIMU.angleUnit = BNO055IMU.AngleUnit.DEGREES; //Sets the unit in which we measure orientation in degrees
        parametersIMU.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC; //Sets acceleration unit in meters per second ??
        parametersIMU.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode, sets what file the IMU ueses
        parametersIMU.loggingEnabled = true; //Sets wether logging in enable
        parametersIMU.loggingTag = "IMU"; //Sets logging tag
        parametersIMU.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator(); //Sets acceleration integration algorithm
        parametersIMU.temperatureUnit = BNO055IMU.TempUnit.CELSIUS; //Sets units for temperature readings
        imu = hardwareMap.get(BNO055IMU.class, "imu"); //Inits IMU
        imu.initialize(parametersIMU); //Init IMU parameters (set above)
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //Gets current orientation of robot
        telemetry.addData("Init Orientation", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)); //Displays initial orientation
        robot.motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DcMotor[] motors = new DcMotor[4];
        motors[0] = robot.motorRF;
        motors[1] = robot.motorRB;
        motors[2] = robot.motorLB;
        motors[3] = robot.motorLF;

        if (isCameraAvailable()) {

            setCameraDownsampling(1);

            startCamera();

            waitForStart();

            robot.motorLift.setPower(1.0);
            while (robot.motorLift.getCurrentPosition() < 10500) { //Was previously 11117
                telemetry.addData("motor lift pos", robot.motorLift.getCurrentPosition());
                telemetry.update();
            }

            robot.motorLift.setPower(0);


            sleep(SLEEP_TIME);

            //drive.timeDrive(500,0.3, driveStyle.BACKWARD, motors); //Robot orientation is forward

                //Lower robot to ground
                /*robot.motorLift.setPower(1.0);
                while (robot.motorLift.getCurrentPosition() <= 10500) {
                    telemetry.addData("motor lift pos", robot.motorLift.getCurrentPosition());
                    telemetry.update();
                }
                robot.motorLift.setPower(0);

                sleep(SLEEP_TIME);
                */

                if (imageReady()) {
                    int redValueLeft = -76800;
                    int blueValueLeft = -76800;
                    int greenValueLeft = -76800;

                    Bitmap rgbImage;
                    //The last value must correspond to the down sampling value from above
                    rgbImage = convertYuvImageToRgb(yuvImage, width, height, 1);


                    //This is for only saving the color image if needed.

                    for (int x = xMin; x <= xMax; x++) {
                        for (int y = yMin; y <= yMax; y++) {
                            if (x == xMax && y <= yMax) {
                                rgbImage.setPixel(x, y, Color.rgb(0, 255, 0));
                            }
                            if (x <= xMax && y == yMin) {
                                rgbImage.setPixel(x, y, Color.rgb(0, 255, 0));
                            }
                            if (x == xMin && y <= yMax) {
                                rgbImage.setPixel(x, y, Color.rgb(0, 255, 0));
                            }
                            if (x <= xMax && y == yMax) {
                                rgbImage.setPixel(x, y, Color.rgb(0, 255, 0));

                            }
                        }
                    }


                    SaveImage(rgbImage);

                    //Analyzing Jewel Color
                    for (int x = xMin; x < xMax; x++) {
                        for (int y = yMin; y < yMax; y++) {
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
                    if (Math.abs((redValueLeft - blueValueLeft)) <= 5) {
                        telemetry.addData("Mineral Color", "0 : Silver");
                        telemetry.addData("Red", redValueLeft);
                        telemetry.addData("Blue", blueValueLeft);
                        telemetry.addData("Green", greenValueLeft);
                        telemetry.addData("Red Blue Difference", Math.abs((redValueLeft - blueValueLeft)));
                    } else {
                        telemetry.addData("Mineral Color", "Gold");
                        telemetry.addData("Red", redValueLeft);
                        telemetry.addData("Blue", blueValueLeft);
                        telemetry.addData("Green", greenValueLeft);
                        mineralPositions = MineralPositions.CENTER;
                        telemetry.addData("Mineral Position", mineralPositions);
                    }
                    telemetry.update();
                    sleep(SLEEP_TIME);

                }

                stopCamera();
            }
            telemetry.addData("Before Correction", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
            telemetry.update();


            //Makes the robot move away from the latch
            drive.encoderDrive(90, driveStyle.BACKWARD, 0.6, motors);
            sleep(SLEEP_TIME);

            //FIRST PICTURE SECOND POSITION
            //turns the robot so it can take a picture of the right mineral
            if (mineralPositions != MineralPositions.CENTER) {
                //second picture
                if (isCameraAvailable()) {

                    startCamera();
                    setCameraDownsampling(1);
                    int counter = 0;
                    while (!imageReady() && counter < 6) {
                        telemetry.addData("Image Not Ready", counter++);
                        telemetry.update();
                        sleep(500);
                    }
                    if (imageReady()) {
                        int redValueLeft = -76800;
                        int blueValueLeft = -76800;
                        int greenValueLeft = -76800;

                        Bitmap rgbImage;
                        //The last value must correspond to the downsampling value from above
                        // rgbImage = convertYuvImageToRgb(yuvImage, width, height, 1);

                        rgbImage = convertYuvImageToRgb(yuvImage, width, height, 1);

                        xMax = 300;
                        xMin = 0;

                        yMin = 925;
                        yMax = 1175;

                        for (int x = xMin; x <= xMax; x++) {
                            for (int y = yMin; y <= yMax; y++) {
                                if (x == xMax && y <= yMax) {
                                    rgbImage.setPixel(x, y, Color.rgb(0, 255, 0));
                                }
                                if (x <= xMax && y == yMin) {
                                    rgbImage.setPixel(x, y, Color.rgb(0, 255, 0));
                                }
                                if (x == xMin && y <= yMax) {
                                    rgbImage.setPixel(x, y, Color.rgb(0, 255, 0));
                                }
                                if (x <= xMax && y == yMax) {
                                    rgbImage.setPixel(x, y, Color.rgb(0, 255, 0));

                                }
                            }
                        }


                        SaveImage(rgbImage);

                        //Analyzing Jewel Color
                        for (int x = xMin; x < xMax; x++) {
                            for (int y = yMin; y < yMax; y++) {
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
                        if (Math.abs((redValueLeft - blueValueLeft)) <= 5) {
                            telemetry.addData("Mineral Color", "1 : Silver");
                            telemetry.addData("Red", redValueLeft);
                            telemetry.addData("Blue", blueValueLeft);
                            telemetry.addData("Green", greenValueLeft);
                            telemetry.addData("Red Blue Difference", Math.abs((redValueLeft - blueValueLeft)));
                            mineralPositions = MineralPositions.RIGHT;
                            telemetry.addData("Mineral Position", mineralPositions);
                        } else {
                            telemetry.addData("Mineral Color", "Gold");
                            telemetry.addData("Red", redValueLeft);
                            telemetry.addData("Blue", blueValueLeft);
                            telemetry.addData("Green", greenValueLeft);
                            mineralPositions = MineralPositions.LEFT;
                            telemetry.addData("Mineral Position", mineralPositions);
                        }
                        telemetry.update();
                        sleep(SLEEP_TIME);

                    } else {
                        telemetry.addData("Image Not Ready", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
                        telemetry.update();
                    }

                    stopCamera();
                }
            }

            sleep(SLEEP_TIME);

            drive.encoderDrive(100, driveStyle.STRAFE_LEFT, 0.8, motors);

            sleep(SLEEP_TIME);


            switch (mineralPositions) {
                case RIGHT: {
                    drive.encoderDrive(185, driveStyle.FORWARD, 0.6, motors);
                    drive.OrientationDrive(70, 0.5, motors, imu);
                    break;
                }

                case LEFT: {
                    drive.OrientationDrive(100, 0.5, motors, imu);
                    break;
                }

                case CENTER: {
                    //drive.encoderDrive(60, driveStyle.FORWARD, 0.6, motors);
                    drive.OrientationDrive(70, 0.5, motors, imu);
                    break;
                }
            }


            sleep(SLEEP_TIME);

            drive.timeDrive(2250, 0.4, driveStyle.FORWARD, motors); //Robot orientation is forward
        }
    }





