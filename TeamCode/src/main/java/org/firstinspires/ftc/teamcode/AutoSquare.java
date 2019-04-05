package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous(name="AutoSquare",group = "")
public class AutoSquare extends LinearOpMode
{
    public void runOpMode()
    {
        Drive_MK2 drive = new Drive_MK2();

        DcMotor motorRF = null;
        DcMotor motorLF = null;
        DcMotor motorRB = null;
        DcMotor motorLB = null;

        motorRF = hardwareMap.dcMotor.get("motorRF");
        motorRB = hardwareMap.dcMotor.get("motorRB");
        motorLF = hardwareMap.dcMotor.get("motorLF");
        motorLB = hardwareMap.dcMotor.get("motorLB");

        //Setting motors to brake when stopped
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        DcMotor[] motors = new DcMotor[4];
        motors[0] = motorRF;
        motors[1] = motorRB;
        motors[2] = motorLB;
        motors[3] = motorLF;

        /*while (opModeIsActive())
        {
            motorLB.setPower(0.5);
            telemetry.addData("MotorLBCurrentPos", motorLB.getCurrentPosition());
            telemetry.update();
        */


        drive.encoderDrive(300,driveStyle.FORWARD,0.8,motors);
        sleep(1000);
        drive.encoderDrive(300,driveStyle.STRAFE_LEFT,0.8,motors);
        sleep(1000);
        drive.encoderDrive(300,driveStyle.BACKWARD,0.8,motors);



    }

}


//drive.encoderDrive(100, driveStyle.STRAFE_LEFT, 0.8, motors);
//sleep(500);