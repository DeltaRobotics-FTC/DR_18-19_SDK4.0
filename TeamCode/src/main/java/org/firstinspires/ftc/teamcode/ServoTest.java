package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by User on 10/5/2017.
 */

@TeleOp (name = "ServoTest", group = "")
public class ServoTest extends LinearOpMode
{

    Servo servo;

    boolean leftDPadState = false;
    boolean rightDPadState = false;

    boolean fastIncrement = true;

    double servoPosition = 0.5;

    @Override
    public void runOpMode()
    {
        servo = hardwareMap.servo.get("servo");


        servo.setPosition(servoPosition);

        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.a)
            {
                fastIncrement = true;
            }

            if(gamepad1.b)
            {
                fastIncrement = false;
            }

            if(gamepad1.dpad_left && !leftDPadState)
            {
                leftDPadState = true;
                if(fastIncrement)
                {
                    servoPosition -= 0.05;
                }
                else
                {
                    servoPosition -= 0.01;
                }
            }

            else if(!gamepad1.dpad_left)
            {
                leftDPadState = false;
            }

            if(gamepad1.dpad_right && !rightDPadState)
            {
                rightDPadState = true;

                if(fastIncrement)
                {
                    servoPosition += 0.05;
                }
                else
                {
                    servoPosition += 0.01;
                }
            }


            else if(!gamepad1.dpad_right)
            {
                rightDPadState = false;
            }

            if(servoPosition > 1.0)
            {
                servoPosition = 1.0;
            }

            if(servoPosition < 0.0)
            {
                servoPosition = 0.0;
            }

            servo.setPosition(servoPosition);

            telemetry.addData("servo Pos", servo.getPosition());
            telemetry.addData("servoPosition Var", servoPosition);
            telemetry.addData("Fast Increment", fastIncrement);
            telemetry.update();

        }

    }
}
