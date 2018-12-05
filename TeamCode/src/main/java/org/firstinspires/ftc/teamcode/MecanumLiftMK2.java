package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by User on 4/19/2018.
 */
@TeleOp(name="MecanumLiftMK2",group = "")
public class MecanumLiftMK2 extends LinearOpMode
{
    //Object for IMU sensor built into the Rev Module
    BNO055IMU imu;

    //Ojecet that stores values of the orientation sensor from the IMU
    Orientation angles;

    //Object of RobotHardware class that allows access to all the component of the robot
    RobotHardware robot = new RobotHardware();

    //Object of the Drive class so we can move the robot easily with one line of code
    Drive_MK2 drive = new Drive_MK2();

    //Value the drive train motor power will be multiplied by to reduce the speed to make the robot more controllable
    double speed = 0.75;
    double liftSpeed = 0.5;
    double armSpeed = 0.5;


    public void runOpMode()
    {

        //Inits robot hardware
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
        /*robot.motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /*robot.motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */


        //Waits till the start button is pressed before moving on in the code
        waitForStart();
        while (opModeIsActive())
        {


            //sets motor power according to joystick input
            /*robot.motorRF.setPower(speed*((-gamepad1.left_stick_y - gamepad1.left_stick_x) - (zScale * gamepad1.right_stick_x)));
            robot.motorLB.setPower(speed*((gamepad1.left_stick_y + gamepad1.left_stick_x) - (zScale * gamepad1.right_stick_x)));
            robot.motorRB.setPower(speed*(-(-gamepad1.left_stick_x + gamepad1.left_stick_y) - (zScale * gamepad1.right_stick_x)));
            robot.motorLF.setPower(speed*((-gamepad1.left_stick_x + gamepad1.left_stick_y)) - (zScale * gamepad1.right_stick_x));
            */

            /*
            Uses the sub-method "setPower" in the drive class to set the motor powers to each of the drive train motors based on the mecanum algorithm
            The values of the two joysticks are passed in to accomplish this
            */
            robot.motorRF.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[0]);
            robot.motorRB.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[1]);
            robot.motorLB.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[2]);
            robot.motorLF.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[3]);

            //this if statement tell the robot if Y is pressed the arm goes up if A is pressed goes down and if nothings pressed it goes nowhere
            if (gamepad1.y)
            {
                robot.motorLift.setPower(liftSpeed);

            }
            else if(gamepad1.a && robot.liftLimitBottom.getState()) //liftLimitBotom.getState() == true is for when the button is not being pressed
            {
                robot.motorLift.setPower(-liftSpeed);

            }
            else
            {
                robot.motorLift.setPower(0);
            }

            //this program controls the marker dispensing system
            if (gamepad1.right_bumper)
            {
                robot.marker.setPosition(0.15);
            }
            else if(gamepad1.right_trigger >= 0.1)
            {
                robot.marker.setPosition(0.8);
            }

            //Sets the drive train speed to the "fast mode" when the X button is pressed
            if(gamepad1.x)
            {
                speed = 0.75;
            }

            //Sets the drive train speed to the "slow mode" when the B button is pressed
            if(gamepad1.b)
            {
                speed = 0.45;
            }
            if(gamepad1.dpad_up)
            {
                liftSpeed = 1;
            }

            if(gamepad1.dpad_down)
            {
                liftSpeed = 0.3;
            }

            if(gamepad2.right_bumper)
            {
                robot.collectionSweeper.setPosition(1.0);
            }

            if(gamepad2.right_trigger > 0.5)
            {
                robot.collectionSweeper.setPosition(0);
            }

            if(gamepad2.back)
            {
                robot.collectionSweeper.setPosition(0.5);
            }

            if(gamepad2.left_bumper)
            {
                robot.collectionGate.setPosition(0.2);
            }

            if(gamepad2.left_trigger > 0.5)
            {
                robot.collectionGate.setPosition(0.75);
            }

            //for mark 2 the arm and the armEXT for the robot
            robot.Arm.setPower(armSpeed*gamepad2.left_stick_y);//arm
            robot.armEXT.setPower(armSpeed*gamepad2 .right_stick_y);//armEXT

            robot.collectionPivot.setPosition(robot.collectionPivotStartPos + (robot.Arm.getCurrentPosition() * 0.0001));

            //Sends data back to driver station
            /*telemetry.addData("Motor RF Power", robot.motorRF.getPower());
            telemetry.addData("motor LF power", robot.motorLF.getPower());
            telemetry.addData("Motor RB power", robot.motorRB.getPower());
            telemetry.addData("Motor LB power", robot.motorLB.getPower());
            telemetry.addData("motor lift power", robot.motorLift.getPower());
            */

            //Sends back the current encoder value for the lift via telemetry
            /*telemetry.addData("RF Actual Power", robot.motorRF.getPower());
            telemetry.addData("RB Actual Power", robot.motorRB.getPower());
            telemetry.addData("LB Actual Power", robot.motorLB.getPower());
            telemetry.addData("LF Actual Power", robot.motorLF.getPower());
            telemetry.addData("RF Desired Power", drive.setPower(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x)[0]);
            telemetry.addData("RB Desired Power", drive.setPower(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x)[1]);
            telemetry.addData("LB Desired Power", drive.setPower(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x)[2]);
            telemetry.addData("LF Desired Power", drive.setPower(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x)[3]);
            telemetry.addData("left stick x", gamepad1.left_stick_x);
            telemetry.addData("left stick y", gamepad1.left_stick_y);
            telemetry.addData("Orientation", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
            */
            //Sends back the the speed variable defined above via telemetry
            telemetry.addData("encoder lift", robot.motorLift.getCurrentPosition());
            telemetry.addData("Drive speed", speed);
            telemetry.addData("liftSpeed", liftSpeed);
            telemetry.addData("lift switch bottom status", robot.liftLimitBottom.getState());
            telemetry.addData("Arm Pos", robot.Arm.getCurrentPosition());
            telemetry.addData("collectionPivot Pos", robot.collectionPivot.getPosition());
            telemetry.addData("collectionGate Pos", robot.collectionGate.getPosition());
            telemetry.addData("collectionSweeper Pos", robot.collectionSweeper.getPosition());

            //Updates telemetry
            telemetry.update();
        }

    }

}
