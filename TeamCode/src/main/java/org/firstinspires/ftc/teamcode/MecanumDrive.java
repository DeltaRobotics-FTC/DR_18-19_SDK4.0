package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by User on 4/19/2018.
 */
@TeleOp(name="MecanumDrive",group = "")
public class MecanumDrive extends LinearOpMode
{
    double zSclae = 1.0;
    double speed = 1.0;

    DcMotor motorRF = null;
    DcMotor motorLF = null;
    DcMotor motorRB = null;
    DcMotor motorLB = null;
    public void runOpMode()
    {
        motorRF = hardwareMap.dcMotor.get("motorRF");
        motorRB = hardwareMap.dcMotor.get("motorRB");
        motorLF = hardwareMap.dcMotor.get("motorLF");
        motorLB = hardwareMap.dcMotor.get("motorLB");

        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive())
        {
            /*
            robot.motorRF.setPower(-gamepad1.right_stick_y);
            robot.motorRB.setPower(-gamepad1.right_stick_y);
            robot.motorLF.setPower(gamepad1.left_stick_y);
            robot.motorLB.setPower(gamepad1.left_stick_y);
            */
            motorRF.setPower(speed*((-gamepad1.left_stick_y - gamepad1.left_stick_x) - (zSclae * gamepad1.right_stick_x)));
            motorLB.setPower(speed*((gamepad1.left_stick_y - gamepad1.left_stick_x) + (zSclae * gamepad1.right_stick_x)));
            motorRB.setPower(speed*(-(-gamepad1.left_stick_x + gamepad1.left_stick_y) - (zSclae * gamepad1.right_stick_x)));
            motorLF.setPower(speed*((-gamepad1.left_stick_x + gamepad1.left_stick_y)) + (zSclae * gamepad1.right_stick_x));

            telemetry.addData("Motor RF Power", motorRF.getPower());
            telemetry.addData("motor LF power", motorLF.getPower());
            telemetry.addData("Motor RB power", motorRB.getPower());
            telemetry.addData("Motor LF power", motorLF.getPower());

            telemetry.update();
        }

    }

}
