package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Drive;

/**
 * Created by User on 4/19/2018.
 */
@Autonomous(name="AutoRedRight",group = "")
public class AutoRedRight extends LinearOpMode
{
    RobotHardware robot = new RobotHardware();

    Drive drive = new Drive();

    int stepSleep = 500;
    public void runOpMode()
    {

        robot.init(hardwareMap);

        robot.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        DcMotor[] motors = new DcMotor[4];
        motors[0] = robot.motorRF;
        motors[1] = robot.motorRB;
        motors[2] = robot.motorLB;
        motors[3] = robot.motorLF;


        waitForStart();

        //Lower robot to ground
        /*
        robot.motorLift.setPower(1.0);
        while(robot.motorLift.getCurrentPosition() <= 20300)
        {
        telemetry.addData("motor lift pos",robot.motorLift.getCurrentPosition());
        }
        robot.motorLift.setPower(0);
        */

        sleep(stepSleep);// wait till next step

        drive.encoderDrive(200,driveStyle.STRAFE_RIGHT,0.20,motors);// makes the robot move right away from the latch

        sleep(stepSleep);// wait till next step

        drive.encoderDrive(1600,driveStyle.BACKWARD,0.30,motors);// moves the robot to the Depot

        sleep(stepSleep);// wait till next step

        robot.marker.setPosition(0.8);// sets the servo to the set drop position

        sleep(stepSleep);// wait till next step

        robot.marker.setPosition(0.15);//setd the servo to the set up position

        sleep(stepSleep);// wait till next step






    }

}