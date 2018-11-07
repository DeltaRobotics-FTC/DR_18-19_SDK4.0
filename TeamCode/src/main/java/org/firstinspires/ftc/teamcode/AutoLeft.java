package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by User on 4/19/2018.
 */
@Autonomous(name="AutoLeft",group = "")
public class AutoLeft extends LinearOpMode
{
    RobotHardware robot = new RobotHardware();
    BNO055IMU imu;
    Orientation angles;

    Drive drive = new Drive();

    int stepSleep = 500;
    double targetError = 0;
    double pivotValue = 0;

        public void runOpMode()
        {

            robot.init(hardwareMap);

            /*BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters(); //Declares parameters object forIMU
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
            */

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


            waitForStart();

            //Lower robot to ground
            robot.motorLift.setPower(1.0);
            while (robot.motorLift.getCurrentPosition() <= 11117)
            {
                telemetry.addData("motor lift pos", robot.motorLift.getCurrentPosition());
                telemetry.update();
            }
            robot.motorLift.setPower(0);


            sleep(stepSleep);// wait till next step


            //Makes the robot move right away from the latch
            double target = (robot.motorRF.getCurrentPosition() - 700);
            robot.motorRF.setPower(drive.setPower(0, 0.65, 0)[0]);
            robot.motorLF.setPower(drive.setPower(0, -0.65, 0)[3]);
            while (robot.motorRF.getCurrentPosition() >= target)
            {
                telemetry.addData("target", target);
                telemetry.addData("RF Pos", robot.motorRF.getCurrentPosition());
                telemetry.update();
            }
            robot.motorRF.setPower(0);
            robot.motorLF.setPower(0);

            /*sleep(stepSleep);// wait till next step

            //Moves the robot away from the lander

            drive.encoderDrive(800, driveStyle.BACKWARD, 0.3, motors);

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

            //Orients robot to its starting orientation (from when it was hooked on lander). This makes sure we are oriented towards the mineral group and correct any errors in the orientation
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //Gets current orientation of robot
            targetError = (pivotValue + AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
            telemetry.addData("Before Correction", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
            telemetry.addData("Target Error", targetError);
            telemetry.update();

            if (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) > (pivotValue + 3) || AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) < (pivotValue - 3))
            {
                if (targetError < 0) //If the robot's current orientation is greater than 0
                {
                    telemetry.addData("Target Error -", targetError);
                    telemetry.update(); //Updates telemetry
                    drive.OrientationDrive(Math.abs(targetError), driveStyle.PIVOT_LEFT, 0.3, motors, imu); //Moves robot to correct orientation
                }

                else //If the robot's current orientation isn't greater than 0
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

                sleep(stepSleep);

                //Moves robot into the depot, knocking off the middle mineral
                drive.encoderDrive(2500, driveStyle.BACKWARD, 0.3, motors);// moves the robot to the Depot

                sleep(stepSleep);// wait till next step

                //Sets the servo to the set drop position
                robot.marker.setPosition(0.8);

                sleep(stepSleep);//wait till next step

                //Set the servo to the set up position
                robot.marker.setPosition(0.15);

                sleep(stepSleep);// wait till next step

                //Moves robot away from the depot
                drive.encoderDrive(550, driveStyle.FORWARD, 0.3, motors);

                sleep(stepSleep);// wait till next step

                //Orients robot so it is close to parallel with the perimeter wall
                drive.OrientationDrive(40, driveStyle.PIVOT_LEFT, 0.3, motors, imu);

                sleep(stepSleep);// wait till next step

                //Strafes towards the perimeter wall so robot will clear the minerals
                drive.encoderDrive(1100, driveStyle.STRAFE_LEFT, 0.6, motors);

                sleep(stepSleep);// wait till next step

                //Corrects robots orientation to verify that it is parallel to the perimeter wall
                pivotValue = 45;
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //Gets current orientation of robot
                targetError = (pivotValue - AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
                telemetry.addData("Before Correction", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
                telemetry.addData("Target Error", targetError);
                telemetry.update();

                if (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) > (pivotValue + 3) || AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) < (pivotValue - 3))
                {
                    if (targetError < 0) //If the robot's current orientation is greater than 0
                    {
                        telemetry.addData("Expected Change 1", targetError);
                        telemetry.update(); //Updates telemetry
                        drive.OrientationDrive(Math.abs(targetError), driveStyle.PIVOT_LEFT, 0.3, motors, imu); //Moves robot to correct orientation
                    }

                    else //If the robot's current orientation isn't greater than 0
                    {
                        telemetry.addData("Expected Change 2", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
                        telemetry.update(); //Updates telemetry
                        drive.OrientationDrive(Math.abs(targetError), driveStyle.PIVOT_RIGHT, 0.3, motors, imu); //Moves robot to correct orientation
                    }
                }
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //Gets current orientation of robot
                telemetry.addData("Target Error", targetError);
                telemetry.addData("After Move", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)); //Displays robot's orientation after the orientation correction
                telemetry.update(); //Updates telemetry

                sleep(stepSleep);

                //Moves robot in or close to the crater
                drive.encoderDrive(4000, driveStyle.FORWARD, 0.5, motors);
                sleep(stepSleep);
                //Moves robot a little bit to verify it is in the crater
                drive.timeDrive(500, 0.25, driveStyle.FORWARD, motors)*/

        }

    }

