package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by User on 4/19/2018.
 */
@TeleOp(name="MecanumLift",group = "")
public class MecanumLift extends LinearOpMode
{

    RobotHardware robot = new RobotHardware();
    Drive drive = new Drive();

    double zScale = 1.0;
    double speed = 1.0;


    public void runOpMode()
    {

        robot.init(hardwareMap);
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


        waitForStart();
        while (opModeIsActive())
        {


            //sets motor power according to joystick input
            /*robot.motorRF.setPower(speed*((-gamepad1.left_stick_y - gamepad1.left_stick_x) - (zScale * gamepad1.right_stick_x)));
            robot.motorLB.setPower(speed*((gamepad1.left_stick_y + gamepad1.left_stick_x) - (zScale * gamepad1.right_stick_x)));
            robot.motorRB.setPower(speed*(-(-gamepad1.left_stick_x + gamepad1.left_stick_y) - (zScale * gamepad1.right_stick_x)));
            robot.motorLF.setPower(speed*((-gamepad1.left_stick_x + gamepad1.left_stick_y)) - (zScale * gamepad1.right_stick_x));
            */

            robot.motorRF.setPower(drive.setPower(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x)[0]);
            robot.motorRB.setPower(drive.setPower(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x)[1]);
            robot.motorLB.setPower(drive.setPower(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x)[2]);
            robot.motorLF.setPower(drive.setPower(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x)[3]);

            //this if statement tell the robot if Y is pressed the arm goes up if A is pressed goes down and if nothings pressed it goes nowhere
            if (gamepad1.y)
            {
                robot.motorLift.setPower(1.0);

            }
            else if(gamepad1.a)
            {
                robot.motorLift.setPower(-1.0);

            }
            else
            {
                robot.motorLift.setPower(0);
            }

            //this progeram controls the marker dispencing system
            if (gamepad1.right_bumper)
            {
                robot.marker.setPosition(0.15);
            }
            else if(gamepad1.right_trigger >= 0.1)
            {
                robot.marker.setPosition(0.8);
            }


            //Sends data back to driver station
            /*telemetry.addData("Motor RF Power", robot.motorRF.getPower());
            telemetry.addData("motor LF power", robot.motorLF.getPower());
            telemetry.addData("Motor RB power", robot.motorRB.getPower());
            telemetry.addData("Motor LB power", robot.motorLB.getPower());
            telemetry.addData("motor lift power", robot.motorLift.getPower());
            */

            telemetry.addData("encoder lift", robot.motorLift.getCurrentPosition());
            telemetry.addData("RF Actual Power", robot.motorRF.getPower());
            telemetry.addData("RB Actual Power", robot.motorRB.getPower());
            telemetry.addData("LB Actual Power", robot.motorLB.getPower());
            telemetry.addData("LF Actual Power", robot.motorLF.getPower());
            telemetry.addData("RF Desired Power", drive.setPower(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x)[0]);
            telemetry.addData("RB Desired Power", drive.setPower(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x)[1]);
            telemetry.addData("LB Desired Power", drive.setPower(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x)[2]);
            telemetry.addData("LF Desired Power", drive.setPower(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x)[3]);
            telemetry.addData("left stick x", gamepad1.left_stick_x);
            telemetry.addData("left stick y", gamepad1.left_stick_y);

            telemetry.update();
        }

    }

}
