package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by User on 4/19/2018.
 */
@TeleOp(name="MecanumLiftMK2",group = "")
public class MecanumLiftMK2 extends LinearOpMode
{
    //Object of RobotHardware class that allows access to all the component of the robot
    RobotHardware_MK2 robot = new RobotHardware_MK2();

    ElapsedTime runtime = new ElapsedTime();

    //Object of the Drive class so we can move the robot easily with one line of code
    Drive_MK2 drive = new Drive_MK2();

    //Value the drive train motor power will be multiplied by to reduce the speed to make the robot more controllable
    double speed = 0.7;
    double armSpeed = 0.85;
    boolean collectionPivotAuto = false;

    final double SCALING_VALUE = 0.02169;

    final int ARM_MAX_POS = -6599;
    final int ARM_MIN_POS = 2393;

    final int ARM_EXT_SCORE = -6500;
    final double PIVOT_SCORE = 0.325;
    final double PIVOT_COLLECT = 0.148;
    final int ARM_COLLECT = 756;
    final int ARM_SCORE = -4928;
    final int ARM_TRAVEL = -900;
    final double PIVOT_TRAVEL = 0.29;

    final double PIVOT_MIN_POS = 0.65;

    double lastTiltManuel = 0;

    boolean dPadUpState = false;
    boolean dPadDownState = false;
    boolean dPadLeftState = false;

    final int PIVOT_STEPS = 10;
    double armStepTarget = 0;
    double armEncDelta = 0;
    double armStepIncr = 0;

    double pivotStepIncr = 0;
    double pivotDelta = 0;

    public void runOpMode()
    {

        //Inits robot hardware
        robot.init(hardwareMap);

        robot.Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armEXT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armEXT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double tiltManuel = robot.collectionPivot.getPosition();



        //Waits till the start button is pressed before moving on in the code

        telemetry.addData("Starting Time", runtime.milliseconds());
        waitForStart();
        while (opModeIsActive())
        {

            robot.motorRF.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[0]);
            robot.motorRB.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[1]);
            robot.motorLB.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[2]);
            robot.motorLF.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[3]);

            //this if statement tell the robot if Y is pressed the arm goes up if A is pressed goes down and if nothings pressed it goes nowhere
            if (gamepad1.y)
            {
                robot.motorLift.setPower(1.0);

            }
            else if(gamepad1.a && robot.liftLimitBottom.getState())
            {
                robot.motorLift.setPower(-1.0);

            }
            else
            {
                robot.motorLift.setPower(0);
            }

            //this program controls the marker dispensing system
            if (gamepad1.right_bumper)
            {
                robot.marker.setPosition(0.15);
            }
            else if(gamepad1.right_trigger >= 0.1)
            {
                robot.marker.setPosition(0.8);
            }

            //Sets the drive train speed to the "fast mode" when the X button is pressed
            if(gamepad1.x)
            {
                speed = 0.6;
            }

            //Sets the drive train speed to the "slow mode" when the B button is pressed
            if(gamepad1.b)
            {
                speed = 0.4;
            }

            // the sweeper starts and stops using the left trigger and bumper
            if(gamepad2.left_bumper)
            {
                robot.collectionSweeper.setPosition(0.5);
            }

            if(gamepad2.left_trigger > 0.5)
            {
                robot.collectionSweeper.setPosition(0);
                robot.collectionGate.setPosition(0.4);
            }

            if(gamepad2.dpad_right)
            {
                robot.collectionSweeper.setPosition(1.0);
            }

            //moves the gate with the right bumper and trigger
            if(gamepad2.right_bumper)
            {
                robot.collectionGate.setPosition(1.0);
            }

            if(gamepad2.right_trigger > 0.5)
            {
                robot.collectionGate.setPosition(0.4);
            }

            //for mark 2 the arm and the armEXT for the robot
            /*if(robot.Arm.getCurrentPosition() >= ARM_MAX_POS && robot.Arm.getCurrentPosition() <= ARM_MIN_POS)
            {

            }
            */
            robot.Arm.setPower(-(armSpeed*gamepad2.left_stick_y));//arm
            robot.armEXT.setPower(gamepad2.right_stick_y);//armEXT

            //is collectionPivotAuto is true runs Auto code
            if (collectionPivotAuto)
                {
                  tiltManuel = ((SCALING_VALUE*(robot.Arm.getCurrentPosition()- ARM_MAX_POS))/1000)+PIVOT_MIN_POS;
                    //robot.collectionPivot.setPosition(((-SCALING_VALUE*(robot.Arm.getCurrentPosition()-ARM_MIN_POS))/1000)+PIVOT_MIN_POS);
                }
            else //if collectionPivotManuel is true it will set the tilt manuel to the desired position using Y for up and A for down
                {
                    if (gamepad2.y) {
                    tiltManuel += 0.002;
                }
                    if (gamepad2.a) {
                        tiltManuel -= 0.002;
                    }
                    }

            if(lastTiltManuel != tiltManuel)
            {
                robot.collectionPivot.setPosition(tiltManuel);
            }



            //changes pivot control from manuel to auto
            if (gamepad2.x) {
                collectionPivotAuto = true;
            }

            //changes pivot control from auto to manuel
            if (gamepad2.b) {
                collectionPivotAuto = false;
            }

            //Scoring
            if(gamepad2.dpad_up && !dPadUpState)
            {
                dPadUpState = true;

                //Arm Extension
                if(robot.armEXT.getCurrentPosition() >= ARM_EXT_SCORE - 100 && robot.armEXT.getCurrentPosition() <= ARM_EXT_SCORE + 100 && !gamepad1.dpad_up)
                {
                    // in range, no action
                }
                else if(robot.armEXT.getCurrentPosition() >= ARM_EXT_SCORE && !gamepad1.dpad_up)
                {// arm too low, move out
                    robot.armEXT.setPower(-0.7);
                    while(robot.armEXT.getCurrentPosition() >= ARM_EXT_SCORE && !gamepad1.dpad_up)
                    {

                        robot.motorRF.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[0]);
                        robot.motorRB.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[1]);
                        robot.motorLB.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[2]);
                        robot.motorLF.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[3]);
                        telemetry.addData("Arm Ext Pos", robot.armEXT.getCurrentPosition());
                        telemetry.update();
                    }
                }
                else if(robot.armEXT.getCurrentPosition() <= ARM_EXT_SCORE && !gamepad1.dpad_up)
                {// arm too high, move in
                    robot.armEXT.setPower(0.5);
                    while(robot.armEXT.getCurrentPosition() <= ARM_EXT_SCORE && !gamepad1.dpad_up)
                    {

                        robot.motorRF.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[0]);
                        robot.motorRB.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[1]);
                        robot.motorLB.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[2]);
                        robot.motorLF.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[3]);
                        telemetry.addData("Arm Ext Pos", robot.armEXT.getCurrentPosition());
                        telemetry.update();
                    }
                }
                robot.armEXT.setPower(0);


                //Arm
                if(robot.Arm.getCurrentPosition() <= ARM_SCORE + 100 && !gamepad1.dpad_up)
                {
                    // in range, no action
                }
                else if(robot.Arm.getCurrentPosition() >= ARM_SCORE && !gamepad1.dpad_up)
                {// arm too low, move up

                    //setup variables
                    armEncDelta = ARM_SCORE - robot.Arm.getCurrentPosition();
                    armStepIncr = armEncDelta / PIVOT_STEPS;
                    pivotDelta = PIVOT_SCORE - tiltManuel;
                    pivotStepIncr = pivotDelta / PIVOT_STEPS;
                    armStepTarget = robot.Arm.getCurrentPosition() + armStepIncr;

                    telemetry.addData("BeforeMove","");
                    telemetry.addData("armStepTarget", armStepTarget);
                    telemetry.addData("armEncDelta", armEncDelta);
                    telemetry.addData("armStepIncr", armStepIncr);
                    telemetry.addData("pivotDelta", pivotDelta);
                    telemetry.addData("pivotStepTarget", pivotStepIncr);
                    telemetry.update();

                    robot.Arm.setPower(-1.0);
                    while(robot.Arm.getCurrentPosition() >= ARM_SCORE && !gamepad1.dpad_up)
                    {

                        if(armEncDelta > 0) //checks direction of movement
                        {
                            if(robot.Arm.getCurrentPosition() >= armStepTarget)
                            {
                                armStepTarget += armStepIncr;
                                tiltManuel += pivotStepIncr;
                                robot.collectionPivot.setPosition(tiltManuel);
                            }
                        }
                        else
                        {
                            if(robot.Arm.getCurrentPosition() <= armStepTarget)
                            {
                                armStepTarget += armStepIncr;
                                tiltManuel += pivotStepIncr;
                                robot.collectionPivot.setPosition(tiltManuel);
                            }
                        }

                        robot.motorRF.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[0]);
                        robot.motorRB.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[1]);
                        robot.motorLB.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[2]);
                        robot.motorLF.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[3]);
                        telemetry.addData("Arm Pos", robot.Arm.getCurrentPosition());
                        telemetry.update();
                    }
                }
                robot.Arm.setPower(0);
                tiltManuel = PIVOT_SCORE;
                robot.collectionPivot.setPosition(tiltManuel);
            }
            else if(!gamepad2.dpad_up)
            {
                dPadUpState = false;
            }

            //Travel
            if(gamepad2.dpad_left && !dPadLeftState)
            {
                dPadLeftState = true;

                //Arm
                if(robot.Arm.getCurrentPosition() >= ARM_TRAVEL - 100 && robot.Arm.getCurrentPosition() <= ARM_TRAVEL + 100 && !gamepad1.dpad_up)
                {
                    // in range, no action
                }
                else if(robot.Arm.getCurrentPosition() >= ARM_TRAVEL && !gamepad1.dpad_up)
                {// arm too low, move up

                    //setup variables
                    armEncDelta = ARM_TRAVEL - robot.Arm.getCurrentPosition();
                    armStepIncr = armEncDelta / PIVOT_STEPS;
                    armStepTarget = robot.Arm.getCurrentPosition() + armStepIncr;
                    pivotDelta = PIVOT_TRAVEL - tiltManuel;
                    pivotStepIncr = pivotDelta / PIVOT_STEPS;

                    telemetry.addData("BeforeMove","");
                    telemetry.addData("armStepTarget", armStepTarget);
                    telemetry.addData("armEncDelta", armEncDelta);
                    telemetry.addData("armStepIncr", armStepIncr);
                    telemetry.addData("pivotDelta", pivotDelta);
                    telemetry.addData("pivotStepTarget", pivotStepIncr);
                    telemetry.update();

                    robot.Arm.setPower(-1.0);
                    while(robot.Arm.getCurrentPosition() >= ARM_TRAVEL && !gamepad1.dpad_up)
                    {
                        if(armEncDelta > 0) //checks direction of movement
                        {
                            if(robot.Arm.getCurrentPosition() >= armStepTarget)
                            {
                               armStepTarget += armStepIncr;
                               tiltManuel += pivotStepIncr;
                                robot.collectionPivot.setPosition(tiltManuel);
                            }
                        }
                        else
                        {
                            if(robot.Arm.getCurrentPosition() <= armStepTarget)
                            {
                                armStepTarget += armStepIncr;
                                tiltManuel += pivotStepIncr;
                                robot.collectionPivot.setPosition(tiltManuel);
                            }
                        }

                        robot.motorRF.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[0]);
                        robot.motorRB.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[1]);
                        robot.motorLB.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[2]);
                        robot.motorLF.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[3]);
                        telemetry.addData("armStepIncr", armStepIncr);
                        telemetry.addData("Arm Pos", robot.Arm.getCurrentPosition());
                        telemetry.update();
                    }
                }
                else if(robot.Arm.getCurrentPosition() <= ARM_TRAVEL && !gamepad1.dpad_up)
                {// arm too high, move down
                    armEncDelta = ARM_TRAVEL - robot.Arm.getCurrentPosition();
                    armStepIncr = armEncDelta / PIVOT_STEPS;
                    armStepTarget = robot.Arm.getCurrentPosition() + armStepIncr;
                    pivotDelta = PIVOT_TRAVEL - tiltManuel;
                    pivotStepIncr = pivotDelta / PIVOT_STEPS;

                    telemetry.addData("BeforeMove","");
                    telemetry.addData("armStepTarget", armStepTarget);
                    telemetry.addData("armEncDelta", armEncDelta);
                    telemetry.addData("armStepIncr", armStepIncr);
                    telemetry.addData("pivotDelta", pivotDelta);
                    telemetry.addData("pivotStepTarget", pivotStepIncr);
                    telemetry.update();
                    robot.Arm.setPower(1.0);
                    while(robot.Arm.getCurrentPosition() <= ARM_TRAVEL && !gamepad1.dpad_up)
                    {
                        if(armEncDelta > 0) //checks direction of movement
                        {
                            if(robot.Arm.getCurrentPosition() >= armStepTarget)
                            {
                                armStepTarget += armStepIncr;
                                tiltManuel += pivotStepIncr;
                                robot.collectionPivot.setPosition(tiltManuel);
                            }
                        }
                        else
                        {
                            if(robot.Arm.getCurrentPosition() <= armStepTarget)
                            {
                                armStepTarget += armStepIncr;
                                tiltManuel += pivotStepIncr;
                                robot.collectionPivot.setPosition(tiltManuel);
                            }
                        }

                        robot.motorRF.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[0]);
                        robot.motorRB.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[1]);
                        robot.motorLB.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[2]);
                        robot.motorLF.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[3]);
                        telemetry.addData("Arm Pos", robot.Arm.getCurrentPosition());
                        telemetry.update();
                    }
                }
                robot.Arm.setPower(0);

                //Arm Extension
                if(robot.armEXT.getCurrentPosition() >= ARM_EXT_SCORE - 100 && robot.armEXT.getCurrentPosition() <= ARM_EXT_SCORE + 100 && !gamepad1.dpad_up)
                {
                    // in range, no action
                }
                else if(robot.armEXT.getCurrentPosition() >= ARM_EXT_SCORE && !gamepad1.dpad_up)
                {// arm too low, move out
                    robot.armEXT.setPower(-0.9);
                    while(robot.armEXT.getCurrentPosition() >= ARM_EXT_SCORE && !gamepad1.dpad_up)
                    {
                        robot.motorRF.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[0]);
                        robot.motorRB.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[1]);
                        robot.motorLB.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[2]);
                        robot.motorLF.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[3]);
                        telemetry.addData("Arm Ext Pos", robot.armEXT.getCurrentPosition());
                        telemetry.update();
                    }
                }
                else if(robot.armEXT.getCurrentPosition() <= ARM_EXT_SCORE && !gamepad1.dpad_up)
                {// arm too high, move in
                    robot.armEXT.setPower(0.7);
                    while(robot.armEXT.getCurrentPosition() <= ARM_EXT_SCORE && !gamepad1.dpad_up)
                    {
                        robot.motorRF.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[0]);
                        robot.motorRB.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[1]);
                        robot.motorLB.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[2]);
                        robot.motorLF.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[3]);
                        telemetry.addData("Arm Ext Pos", robot.armEXT.getCurrentPosition());
                        telemetry.update();
                    }
                }
                robot.armEXT.setPower(0);

                /*tiltManuel = PIVOT_TRAVEL;
                robot.collectionPivot.setPosition(tiltManuel);
                */
            }
            else if(!gamepad2.dpad_left)
            {
                dPadLeftState = false;
            }

            //Collection
            if(gamepad2.dpad_down && !dPadDownState)
            {
                dPadDownState = true;
                //tiltManuel = PIVOT_COLLECT;
                robot.collectionGate.setPosition(0.4);

                //Arm Extension
                if(robot.armEXT.getCurrentPosition() >= ARM_EXT_SCORE - 100 && robot.armEXT.getCurrentPosition() <= ARM_EXT_SCORE + 100 && !gamepad1.dpad_up)
                {
                    // in range, no action
                }
                else if(robot.armEXT.getCurrentPosition() >= ARM_EXT_SCORE && !gamepad1.dpad_up)
                {// arm too low, move out
                    robot.armEXT.setPower(-0.7);
                    while(robot.armEXT.getCurrentPosition() >= ARM_EXT_SCORE && !gamepad1.back)
                    {
                        robot.motorRF.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[0]);
                        robot.motorRB.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[1]);
                        robot.motorLB.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[2]);
                        robot.motorLF.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[3]);
                        telemetry.addData("Arm Ext Pos", robot.armEXT.getCurrentPosition());
                        telemetry.update();
                    }
                }
                else if(robot.armEXT.getCurrentPosition() <= ARM_EXT_SCORE && !gamepad1.dpad_up)
                {// arm too high, move in
                    robot.armEXT.setPower(0.5);
                    while(robot.armEXT.getCurrentPosition() <= ARM_EXT_SCORE && !gamepad1.back)
                    {
                        robot.motorRF.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[0]);
                        robot.motorRB.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[1]);
                        robot.motorLB.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[2]);
                        robot.motorLF.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[3]);
                        telemetry.addData("Arm Ext Pos", robot.armEXT.getCurrentPosition());
                        telemetry.update();
                    }
                }
                robot.armEXT.setPower(0);

                //Arm
                if(robot.Arm.getCurrentPosition() >= ARM_COLLECT - 100)
                {
                    // in range, no action
                }
                /*else if(robot.Arm.getCurrentPosition() >= ARM_COLLECT && !gamepad1.dpad_up)
                {// arm too low, move up
                    robot.Arm.setPower(-1.0);
                    while(robot.Arm.getCurrentPosition() >= ARM_COLLECT && !gamepad1.dpad_up)
                    {
                        robot.motorRF.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[0]);
                        robot.motorRB.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[1]);
                        robot.motorLB.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[2]);
                        robot.motorLF.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[3]);
                        telemetry.addData("Arm Pos", robot.Arm.getCurrentPosition());
                        telemetry.update();
                    }
                }
                */
                else if(robot.Arm.getCurrentPosition() <= ARM_COLLECT && !gamepad1.dpad_up)
                {// arm too high, move down

                    armEncDelta = ARM_COLLECT - robot.Arm.getCurrentPosition();
                    armStepIncr = armEncDelta / PIVOT_STEPS;
                    armStepTarget = robot.Arm.getCurrentPosition() + armStepIncr;
                    pivotDelta = PIVOT_COLLECT - tiltManuel;
                    pivotStepIncr = pivotDelta / PIVOT_STEPS;

                    telemetry.addData("BeforeMove","");
                    telemetry.addData("armStepTarget", armStepTarget);
                    telemetry.addData("armEncDelta", armEncDelta);
                    telemetry.addData("armStepIncr", armStepIncr);
                    telemetry.addData("pivotDelta", pivotDelta);
                    telemetry.addData("pivotStepTarget", pivotStepIncr);
                    telemetry.update();
                    robot.Arm.setPower(1.0);
                    while(robot.Arm.getCurrentPosition() <= ARM_COLLECT && !gamepad1.dpad_up)
                    {
                        if(armEncDelta > 0) //checks direction of movement
                        {
                            if(robot.Arm.getCurrentPosition() >= armStepTarget)
                            {
                                armStepTarget += armStepIncr;
                                tiltManuel += pivotStepIncr;
                                robot.collectionPivot.setPosition(tiltManuel);
                            }
                        }
                        else
                        {
                            if(robot.Arm.getCurrentPosition() <= armStepTarget)
                            {
                                armStepTarget += armStepIncr;
                                tiltManuel += pivotStepIncr;
                                robot.collectionPivot.setPosition(tiltManuel);
                            }
                        }

                        robot.motorRF.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[0]);
                        robot.motorRB.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[1]);
                        robot.motorLB.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[2]);
                        robot.motorLF.setPower(speed * drive.setPower(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x)[3]);
                        telemetry.addData("Arm Pos", robot.Arm.getCurrentPosition());
                        telemetry.update();
                    }
                }
                robot.Arm.setPower(0);
            }
            else if(!gamepad2.dpad_down)
            {
                dPadDownState = false;
            }



            lastTiltManuel = tiltManuel;


            //Sends back the the speed variable defined above via telemetry

            telemetry.addData("encoder lift", robot.motorLift.getCurrentPosition());
            telemetry.addData("Drive speed", speed);
            telemetry.addData("lift switch bottom status", robot.liftLimitBottom.getState());
            telemetry.addData("Arm Pos", robot.Arm.getCurrentPosition());
            telemetry.addData("collectionPivot Pos", robot.collectionPivot.getPosition());
            telemetry.addData("collectionGate Pos", robot.collectionGate.getPosition());
            telemetry.addData("collectionSweeper Pos", robot.collectionSweeper.getPosition());
            telemetry.addData("armExt Pos", robot.armEXT.getCurrentPosition());


            //Updates telemetry
            telemetry.update();
        }

    }

}
