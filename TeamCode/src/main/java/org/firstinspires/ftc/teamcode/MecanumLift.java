package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by User on 4/19/2018.
 */
@TeleOp(name="MecanumLift",group = "")
public class MecanumLift extends LinearOpMode
{
    double zScale = 1.0;
    double speed = 1.0;

    //This sets the motorXX to DcMotor Objects
    DcMotor motorRF = null;
    DcMotor motorLF = null;
    DcMotor motorRB = null;
    DcMotor motorLB = null;
    DcMotor motorLift = null;

    public void runOpMode()
    {
        //Maps motor objects to name in robot configuration
        motorRF = hardwareMap.dcMotor.get("motorRF");
        motorRB = hardwareMap.dcMotor.get("motorRB");
        motorLF = hardwareMap.dcMotor.get("motorLF");
        motorLB = hardwareMap.dcMotor.get("motorLB");
        motorLift = hardwareMap.dcMotor.get("motorLift");




        //Setting motors to brake when stopped
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive())
        {

            //sets motor power according to joystick input
            motorRF.setPower(speed*((-gamepad1.left_stick_y - gamepad1.left_stick_x) - (zScale * gamepad1.right_stick_x)));
            motorLB.setPower(speed*((gamepad1.left_stick_y - gamepad1.left_stick_x) + (zScale * gamepad1.right_stick_x)));
            motorRB.setPower(speed*(-(-gamepad1.left_stick_x + gamepad1.left_stick_y) - (zScale * gamepad1.right_stick_x)));
            motorLF.setPower(speed*((-gamepad1.left_stick_x + gamepad1.left_stick_y)) + (zScale * gamepad1.right_stick_x));

            //this if statement tell the robot if Y is pressed the arm goes up if A is pressed goes down and if nothings pressed it goes nowhere
            if (gamepad1.y)
            {
                motorLift.setPower(-1.0);

            }
            else if(gamepad1.a)
            {
            motorLift.setPower(1.0);

            }
            else
            {
            motorLift.setPower(0);
            }




            //Sends data back to driver station
            telemetry.addData("Motor RF Power", motorRF.getPower());
            telemetry.addData("motor LF power", motorLF.getPower());
            telemetry.addData("Motor RB power", motorRB.getPower());
            telemetry.addData("Motor LF power", motorLF.getPower());
            telemetry.addData("motor lift power", motorLift.getPower());

            telemetry.update();
        }

    }

}
