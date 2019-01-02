package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

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
public class MK2AutoUnhook extends LinearOpModeCamera {
    RobotHardware_MK2 robot = new RobotHardware_MK2();
    Drive_MK2 drive = new Drive_MK2();

    public void runOpMode()
    {
        robot.init(hardwareMap);
        robot.motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DcMotor[] motors = new DcMotor[4];
        motors[0] = robot.motorRF;
        motors[1] = robot.motorRB;
        motors[2] = robot.motorLB;
        motors[3] = robot.motorLF;

        waitForStart();

         robot.motorLift.setPower(1.0);
            while (robot.motorLift.getCurrentPosition() <= 11117) {
                telemetry.addData("motor lift pos", robot.motorLift.getCurrentPosition());
                telemetry.update();
            }
            robot.motorLift.setPower(0);

            sleep(250);

            drive.encoderDrive(45, driveStyle.BACKWARD,0.3, motors);
    }
}

