package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import for_camera_opmodes.LinearOpModeCamera;

/**
 * Created by User on 4/19/2018.
 */

enum MineralPositions
{
    LEFT, CENTER, RIGHT;
}

@Autonomous(name="AutoRight",group = "")
public class AutoRight extends LinearOpModeCamera {
        RobotHardware robot = new RobotHardware();
        BNO055IMU imu;
        Orientation angles;

        Drive drive = new Drive();

        MineralPositions mineralPositions = MineralPositions.LEFT;

        int stepSleep = 250;
        double targetError = 0;
        //double pivotValue = 0;


        int mineralColorInt;


        //Min and max values for the bounds of the area of the image that will be analyzed
        int xMin = 250;
        int xMax = 625;

        int yMin = 975;
        int yMax = 1175;

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

            /*robot.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/
            robot.motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            /*robot.motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
            robot.motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //Declares an array of the drive train motors so they can be passed into the drive class
            DcMotor[] motors = new DcMotor[4];
            motors[0] = robot.motorRF;
            motors[1] = robot.motorRB;
            motors[2] = robot.motorLB;
            motors[3] = robot.motorLF;


            if(isCameraAvailable()) {

                setCameraDownsampling(1);

                startCamera();

                waitForStart();

                //Lower robot to ground
                /*robot.motorLift.setPower(1.0);
                while (robot.motorLift.getCurrentPosition() <= 11117) {
                    telemetry.addData("motor lift pos", robot.motorLift.getCurrentPosition());
                    telemetry.update();
                }
                robot.motorLift.setPower(0);

                sleep(stepSleep);
                */


                if (imageReady()) {
                    int redValueLeft = -76800;
                    int blueValueLeft = -76800;
                    int greenValueLeft = -76800;

                    Bitmap rgbImage;
                    //The last value must correspond to the downsampling value from above
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
                    } else
                    {
                        telemetry.addData("Mineral Color", "Gold");
                        telemetry.addData("Red", redValueLeft);
                        telemetry.addData("Blue", blueValueLeft);
                        telemetry.addData("Green", greenValueLeft);
                        mineralPositions = MineralPositions.CENTER;
                        telemetry.addData("Mineral Position", mineralPositions);
                    }
                    telemetry.update();
                    sleep(stepSleep);

                }

                stopCamera();
            }



            //Makes the robot move right away from the latch
            /*double target = (robot.motorRF.getCurrentPosition() - 700);
            robot.motorRF.setPower(drive.setPower(0, 0.65, 0)[0]);
            robot.motorLF.setPower(drive.setPower(0, -0.65, 0)[3]);
            while (robot.motorRF.getCurrentPosition() >= target) {
                telemetry.addData("target", target);
                telemetry.addData("RF Pos", robot.motorRF.getCurrentPosition());
                telemetry.update();
            }
            */
            robot.motorRF.setPower(drive.setPower(0, 0.65, 0)[0]);
            robot.motorLF.setPower(drive.setPower(0, -0.65, 0)[3]);
            sleep(750);
            robot.motorRF.setPower(0);
            robot.motorLF.setPower(0);

            if(isCameraAvailable() && !(mineralPositions == MineralPositions.CENTER)) {

                startCamera();
                setCameraDownsampling(1);
                if(imageReady()) {
                    int redValueLeft = -76800;
                    int blueValueLeft = -76800;
                    int greenValueLeft = -76800;

                    Bitmap rgbImage;
                    //The last value must correspond to the downsampling value from above
                   //rgbImage = convertYuvImageToRgb(yuvImage, width, height, 1);

                    rgbImage = convertYuvImageToRgb(yuvImage, width, height, 1);

                    xMax = 845;
                    xMin = 290;

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
                        /*for (int x = xMin; x < xMax; x++) {
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
                        telemetry.update();


                        mineralColorInt = highestColor(redValueLeft, blueValueLeft, greenValueLeft);

                        telemetry.addData("Mineral", mineralColorInt);
                        if ((redValueLeft - blueValueLeft) <= 5) {
                            telemetry.addData("Mineral Color", "0 : Gold");
                            telemetry.addData("Red", redValueLeft);
                            telemetry.addData("Blue", blueValueLeft);
                            telemetry.addData("Green", greenValueLeft);
                        } else
                        {
                            telemetry.addData("Mineral Color", "Silver");
                            telemetry.addData("Red", redValueLeft);
                            telemetry.addData("Blue", blueValueLeft);
                            telemetry.addData("Green", greenValueLeft);
                        }
                        telemetry.update();
                        sleep(1000)*/

                }

                stopCamera();
            }

            sleep(stepSleep);

            if(isCameraAvailable() && !(mineralPositions == MineralPositions.CENTER)) {

                startCamera();
                setCameraDownsampling(1);
                if(imageReady()) {
                    int redValueLeft = -76800;
                    int blueValueLeft = -76800;
                    int greenValueLeft = -76800;

                    Bitmap rgbImage;
                    //The last value must correspond to the downsampling value from above
                    //rgbImage = convertYuvImageToRgb(yuvImage, width, height, 1);

                    rgbImage = convertYuvImageToRgb(yuvImage, width, height, 1);

                    xMax = 845;
                    xMin = 290;

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
                    } else
                    {
                        telemetry.addData("Mineral Color", "Gold");
                        telemetry.addData("Red", redValueLeft);
                        telemetry.addData("Blue", blueValueLeft);
                        telemetry.addData("Green", greenValueLeft);
                        mineralPositions = MineralPositions.LEFT;
                        telemetry.addData("Mineral Position", mineralPositions);
                    }
                    telemetry.update();
                    sleep(stepSleep);

                }

                stopCamera();
            }

            //Moves the robot away from the lander
            drive.encoderDrive(900, driveStyle.BACKWARD, 0.3, motors);

            sleep(stepSleep);

            // this straightens the robot back to the Depot
            /*double targetTwo = ( robot.motorRF.getCurrentPosition() + 400);
            robot.motorRF.setPower(drive.setPower(0, -0.5, 0)[0]);
            robot.motorLF.setPower(drive.setPower(0, 0.5, 0)[3]);
            while (robot.motorRF.getCurrentPosition() <= targetTwo)
            {
                telemetry.addData("targetTwo", targetTwo);
                telemetry.addData("RF Pos", robot.motorRF.getCurrentPosition());
                telemetry.update();
            }
            robot.motorRF.setPower(0);
            robot.motorLF.setPower(0);
            */

            //Orients robot to its starting orientation (from when it was hooked on lander). This makes sure we are oriented towards the mineral group and correct any errors in the orientation
            /*angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //Gets current orientation of robot
            targetError = (pivotValue + AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
            telemetry.addData("Before Correction", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
            telemetry.addData("Target Error", targetError);
            telemetry.update();

            if (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) > (pivotValue + 3) || AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) < (pivotValue - 3)) {
                if (targetError < 0) //If the robot's current orientation is greater than 0
                {
                    telemetry.addData("Target Error -", targetError);
                    telemetry.update(); //Updates telemetry
                    drive.OrientationDrive(Math.abs(targetError), driveStyle.PIVOT_LEFT, 0.5, motors, imu); //Moves robot to correct orientation
                } else //If the robot's current orientation isn't greater than 0
                {
                    telemetry.addData("Target Error +", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
                    telemetry.update(); //Updates telemetry
                    drive.OrientationDrive(Math.abs(targetError), driveStyle.PIVOT_RIGHT, 0.5, motors, imu); //Moves robot to correct orientation
                }
            }

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //Gets current orientation of robot
            telemetry.addData("Target Error", targetError);
            telemetry.addData("After Move", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)); //Displays robot's orientation after the orientation correction
            telemetry.update(); //Updates telemetry
            */

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Before Move", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
            drive.OrientationDrive(0, 0.4, motors, imu);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("After Move", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)); //Displays robot's orientation after the orientation correction
            telemetry.update();
            sleep(2500);



            sleep(stepSleep);

            switch (mineralPositions)
            {
                case CENTER:
                {
                    break;
                }

                case LEFT:
                {
                    drive.encoderDrive(1500, driveStyle.STRAFE_LEFT, 0.8, motors);
                    sleep(stepSleep);
                    /*pivotValue = 25;
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //Gets current orientation of robot
                    targetError = (pivotValue + AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
                    telemetry.addData("Before Correction", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
                    telemetry.addData("Target Error", targetError);
                    telemetry.update();

                    if (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) > (pivotValue + 3) || AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) < (pivotValue - 3)) {
                        if (targetError > 0) //If the robot's current orientation is greater than 0
                        {
                            telemetry.addData("Target Error -", targetError);
                            telemetry.update(); //Updates telemetry
                            drive.OrientationDrive(Math.abs(targetError), driveStyle.PIVOT_LEFT, 0.3, motors, imu); //Moves robot to correct orientation
                        } else //If the robot's current orientation isn't greater than 0
                        {
                            telemetry.addData("Target Error +", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
                            telemetry.update(); //Updates telemetry
                            drive.OrientationDrive(Math.abs(targetError), driveStyle.PIVOT_RIGHT, 0.3, motors, imu); //Moves robot to correct orientation
                        }
                    }
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //Gets current orientation of robot
                    telemetry.addData("Target Error", targetError);
                    telemetry.addData("After Move", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)); //Displays robot's orientation after the orientation correction
                    telemetry.update(); //Updates telemetry
                    */
                    drive.OrientationDrive(25, 0.5, motors, imu);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addData("After Move", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)); //Displays robot's orientation after the orientation correction
                    telemetry.update();
                    sleep(2500);


                    break;
                }

                case RIGHT:
                {
                    /*pivotValue = -7;
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //Gets current orientation of robot
                    targetError = (pivotValue - AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
                    telemetry.addData("Before Correction", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
                    telemetry.addData("Target Error", targetError);
                    telemetry.update();

                    if (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) > (pivotValue + 1) || AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) < (pivotValue - 1)) {
                        if (targetError > 0) //If the robot's current orientation is greater than 0
                        {
                            telemetry.addData("Target Error -", targetError);
                            telemetry.update(); //Updates telemetry
                            drive.OrientationDrive(Math.abs(targetError), driveStyle.PIVOT_LEFT, 0.5, motors, imu); //Moves robot to correct orientation
                        } else //If the robot's current orientation isn't greater than 0
                        {
                            telemetry.addData("Target Error +", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
                            telemetry.update(); //Updates telemetry
                            drive.OrientationDrive(Math.abs(targetError), driveStyle.PIVOT_RIGHT, 0.5, motors, imu); //Moves robot to correct orientation
                        }
                    }

                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //Gets current orientation of robot
                    telemetry.addData("Target Error", targetError);
                    telemetry.addData("After Move", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)); //Displays robot's orientation after the orientation correction
                    telemetry.update(); //Updates telemetry

                    sleep(stepSleep);
                    */


                    drive.OrientationDrive(-7, 0.5, motors, imu);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addData("After Move", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)); //Displays robot's orientation after the orientation correction
                    telemetry.update();
                    sleep(2500);
                    drive.encoderDrive(1600, driveStyle.STRAFE_RIGHT, 1.0, motors);



                    break;
                }
            }

            sleep(stepSleep);

            //Moves robot into the depot, knocking off the middle mineral
            //drive.encoderDrive(1250, driveStyle.BACKWARD, 0.3, motors);// moves the robot to the Depot

            switch (mineralPositions)
            {
                case CENTER:
                {
                    drive.encoderDrive(2500, driveStyle.BACKWARD, 0.3, motors);// moves the robot to the Depot
                    break;
                }

                case LEFT:
                {
                    drive.encoderDrive(2200, driveStyle.BACKWARD, 0.3, motors);// moves the robot to the Depot
                    break;
                }

                case RIGHT:
                {
                    drive.encoderDrive(2100, driveStyle.BACKWARD, 0.3, motors);// moves the robot to the Depot
                    break;
                }
            }

                sleep(stepSleep);// wait till next step

                //Moves robot into the depot, knocking off the middle mineral


                //Sets the servo to the set drop position
                robot.marker.setPosition(1.0);

                sleep(1500);//wait till next step

                //Set the servo to the set up position
                robot.marker.setPosition(0.15);

                sleep(stepSleep);// wait till next step

                //Moves robot away from the depot
                drive.encoderDrive(550, driveStyle.FORWARD, 0.3, motors);

                sleep(stepSleep);// wait till next step

                //Orients robot so it is close to parallel with the perimeter wall
                //drive.OrientationDrive(40, driveStyle.PIVOT_LEFT, 0.3, motors, imu);

                /*pivotValue = 35;
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //Gets current orientation of robot
                targetError = (pivotValue - AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
                telemetry.addData("Before Correction", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
                telemetry.addData("Target Error", targetError);
                telemetry.update();
                //sleep(2500);

                if (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) > (pivotValue + 1) || AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) < (pivotValue - 1)) {
                    if (targetError > 0) //If the robot's current orientation is greater than 0
                    {
                        telemetry.addData("Expected Change 1", targetError);
                        telemetry.update(); //Updates telemetry
                        drive.OrientationDrive(Math.abs(targetError), driveStyle.PIVOT_LEFT, 0.5, motors, imu); //Moves robot to correct orientation
                    } else //If the robot's current orientation isn't greater than 0
                    {
                        telemetry.addData("Expected Change 2", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
                        telemetry.update(); //Updates telemetry
                        drive.OrientationDrive(Math.abs(targetError), driveStyle.PIVOT_RIGHT, 0.5, motors, imu); //Moves robot to correct orientation
                    }
                }
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //Gets current orientation of robot
                telemetry.addData("Target Error", targetError);
                telemetry.addData("After Move", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)); //Displays robot's orientation after the orientation correction
                telemetry.update(); //Updates telemetry
                */
                drive.OrientationDrive(35, 0.5, motors, imu);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("After Move", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)); //Displays robot's orientation after the orientation correction
                telemetry.update();
                sleep(2500);


            sleep(stepSleep);// wait till next step

                //Strafes towards the perimeter wall so robot will clear the minerals

                if(mineralPositions == MineralPositions.RIGHT)
                {
                    drive.encoderDrive(200, driveStyle.FORWARD, 0.3, motors);
                    sleep(stepSleep);
                    drive.encoderDrive(1700, driveStyle.STRAFE_LEFT, 0.6, motors);
                }
                else
                {
                    drive.encoderDrive(1100, driveStyle.STRAFE_LEFT, 0.6, motors);
                }


                sleep(stepSleep);// wait till next step

                //Corrects robots orientation to verify that it is parallel to the perimeter wall
                /*pivotValue = 35;
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //Gets current orientation of robot
                targetError = (pivotValue - AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
                telemetry.addData("Before Correction", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
                telemetry.addData("Target Error", targetError);
                telemetry.update();

                if (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) > (pivotValue + 1) || AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) < (pivotValue - 1)) {
                    if (targetError > 0) //If the robot's current orientation is greater than 0
                    {
                        telemetry.addData("Expected Change 1", targetError);
                        telemetry.update(); //Updates telemetry
                        drive.OrientationDrive(Math.abs(targetError), driveStyle.PIVOT_LEFT, 0.5, motors, imu); //Moves robot to correct orientation
                    } else //If the robot's current orientation isn't greater than 0
                    {
                        telemetry.addData("Expected Change 2", targetError);
                        telemetry.update(); //Updates telemetry
                        drive.OrientationDrive(Math.abs(targetError), driveStyle.PIVOT_RIGHT, 0.5, motors, imu); //Moves robot to correct orientation
                    }
                }
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //Gets current orientation of robot
                telemetry.addData("Target Error", targetError);
                telemetry.addData("After Move", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)); //Displays robot's orientation after the orientation correction
                telemetry.update(); //Updates telemetry

                sleep(stepSleep);
                */

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("After Move", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)); //Displays robot's orientation after the orientation correction
                sleep(2500);
                drive.OrientationDrive(35, 0.5, motors, imu);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("After Move", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)); //Displays robot's orientation after the orientation correction
                telemetry.update();
                sleep(2500);


            //Moves robot in or close to the crater
                if(mineralPositions == MineralPositions.LEFT)
                {
                    //drive.encoderDrive(3500, driveStyle.FORWARD, 0.5, motors);
                    drive.timeDrive(4000, 0.5, driveStyle.FORWARD, motors);
                }
                else
                {
                    drive.encoderDrive(4000, driveStyle.FORWARD, 0.5, motors);
                }
                sleep(stepSleep);
                //Moves robot a little bit to verify it is in the crater
                drive.timeDrive(1000, 0.25, driveStyle.FORWARD, motors);






    }
}