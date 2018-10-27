package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Drive;

/**
 * Created by User on 4/19/2018.
 */
@Autonomous(name="AutoRedRight",group = "")
public class AutoRedRight extends LinearOpMode
{
    RobotHardware robot = new RobotHardware();
    BNO055IMU imu;
    Orientation angles;

    Drive drive = new Drive();

    int stepSleep = 500;
    public void runOpMode()
    {

        robot.init(hardwareMap);

        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters(); //Declares parameters object forIMU
        parametersIMU.angleUnit           = BNO055IMU.AngleUnit.DEGREES; //Sets the unit in which we measure orientation in degrees
        parametersIMU.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC; //Sets acceleration unit in meters per second ??
        parametersIMU.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode, sets what file the IMU ueses
        parametersIMU.loggingEnabled      = true; //Sets wether logging in enable
        parametersIMU.loggingTag          = "IMU"; //Sets logging tag
        parametersIMU.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator(); //Sets acceleration integration algorithm
        parametersIMU.temperatureUnit     = BNO055IMU.TempUnit.CELSIUS; //Sets units for temperature readings
        imu = hardwareMap.get(BNO055IMU.class, "imu"); //Inits IMU
        imu.initialize(parametersIMU); //Init IMU parameters (set above)
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //Gets current orientation of robot
        telemetry.addData("Init Orientation", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)); //Displays initial orientation

        /*robot.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);++++++++++++++++++++++++++++++++++++++++++++
        robot.motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/


        DcMotor[] motors = new DcMotor[4];
        motors[0] = robot.motorRF;
        motors[1] = robot.motorRB;
        motors[2] = robot.motorLB;
        motors[3] = robot.motorLF;


        waitForStart();

        //Lower robot to ground

        robot.motorLift.setPower(1.0);
        while(robot.motorLift.getCurrentPosition() <= 10400)
        telemetry.update();
        {
        telemetry.addData("motor lift pos",robot.motorLift.getCurrentPosition());
        }
        robot.motorLift.setPower(0);


        sleep(stepSleep);// wait till next step

       // drive.encoderDrive(350,driveStyle.STRAFE_RIGHT,0.8,motors);// makes the robot move right away from the latch
        double target = ( robot.motorRF.getCurrentPosition() - 350);
        robot.motorRF.setPower(drive.setPower(0, 0.5, 0)[0]);
        robot.motorLF.setPower(drive.setPower(0, -0.5, 0)[3]);
        while (robot.motorRF.getCurrentPosition() >= target)
        {
            telemetry.addData("target", target);
            telemetry.addData("RF Pos", robot.motorRF.getCurrentPosition());
            telemetry.update();
        }
        robot.motorRF.setPower(0);
        robot.motorLF.setPower(0);

        sleep(stepSleep);// wait till next step

        drive.encoderDrive(800,driveStyle.BACKWARD,0.3,motors);

        sleep(stepSleep);

        // this straightens the robot back to the Depot
        double targetTwo = ( robot.motorRF.getCurrentPosition() + 400);
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

        sleep(stepSleep);

        drive.encoderDrive(2500,driveStyle.BACKWARD,0.3,motors);// moves the robot to the Depot

        sleep(stepSleep);// wait till next step

        robot.marker.setPosition(0.8);// sets the servo to the set drop position

        sleep(stepSleep);// wait till next step

        robot.marker.setPosition(0.15);//setd the servo to the set up position

        sleep(stepSleep);// wait till next step

        drive.encoderDrive(550,driveStyle.FORWARD,0.3,motors);

        sleep(stepSleep);// wait till next step

        drive.OrientationDrive(50,driveStyle.PIVOT_LEFT,0.3,motors,imu);

        sleep(stepSleep);// wait till next step

        drive.encoderDrive(900,driveStyle.STRAFE_LEFT,0.85,motors);

        sleep(stepSleep);// wait till next step

        drive.encoderDrive(3800,driveStyle.FORWARD,0.8,motors);






    }

}
