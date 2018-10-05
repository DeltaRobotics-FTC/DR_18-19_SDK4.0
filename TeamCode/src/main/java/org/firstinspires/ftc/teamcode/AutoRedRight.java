package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Drive;

/**
 * Created by User on 4/19/2018.
 */
@TeleOp(name="AutoRedRight",group = "")
public class AutoRedRight extends LinearOpMode
{
    RobotHardware robot = new RobotHardware();

    Drive drive = new Drive();


    public void runOpMode()
    {

        robot.init(hardwareMap);

        DcMotor[] motors = new DcMotor[4];
        motors[0] = robot.motorRF;
        motors[1] = robot.motorRB;
        motors[2] = robot.motorLB;
        motors[3] = robot.motorLF;


        waitForStart();

        robot.motorLift.setPower(1.0);
        while(robot.motorLift.getCurrentPosition() <= 1000)
        {
        telemetry.addData("motor lift pos",robot.motorLift.getCurrentPosition());
        }
        robot.motorLift.setPower(0);


    }

}
