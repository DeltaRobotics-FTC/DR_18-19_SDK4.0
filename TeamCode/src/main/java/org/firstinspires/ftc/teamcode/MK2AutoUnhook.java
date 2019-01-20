package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.media.tv.TvInputManager;

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


@Autonomous(name="MK2AutoUnhook",group = "")
//@Disabled
public class MK2AutoUnhook extends LinearOpModeCamera
{
    BNO055IMU imu;
    Orientation angles;

    RobotHardware_MK2 robot = new RobotHardware_MK2();
    Drive_MK2 drive = new Drive_MK2();

    final int SLEEP_TIME = 500;

    public void runOpMode()
    {
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

        waitForStart();

         robot.motorLift.setPower(1.0);
            while (robot.motorLift.getCurrentPosition() < 10000) { //Was previously 11117
                telemetry.addData("motor lift pos", robot.motorLift.getCurrentPosition());
                telemetry.update();
            }

            robot.motorLift.setPower(0);

            sleep(SLEEP_TIME);

            //drive.timeDrive(500,0.3, driveStyle.BACKWARD, motors); //Robot orientation is forward
            drive.encoderDrive(90, driveStyle.BACKWARD,0.3, motors);

            sleep(SLEEP_TIME);

            drive.encoderDrive(100, driveStyle.STRAFE_LEFT, 0.8, motors);

            sleep(SLEEP_TIME);

            drive.encoderDrive(105, driveStyle.FORWARD, 0.3, motors);

            sleep(SLEEP_TIME);

            drive.OrientationDrive(75, 0.5, motors, imu);

            sleep(SLEEP_TIME);

            drive.timeDrive(2250, 0.4, driveStyle.FORWARD, motors); //Robot orientation is forward
    }
}

