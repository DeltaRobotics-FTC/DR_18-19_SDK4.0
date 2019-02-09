package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * Created by User on 10/7/2017.
 */

//Enumeration of different directions (styles) the robot can drive. These are used as arguments in the drive methods below

public class Drive_MK2 extends LinearOpMode
{

   /*public static double drivePower = 0.75;
    public static double strafePower = 0.85;
    public static double pivotPower = 0.6;
    public static boolean isMethodShareForwardFinished = false;
    public static boolean isMethodShareEncoderTestFinished = false;
    public static double time = 0;
    public static double encoder = 0;
    public static double startingEncoder = 0;
    */

   /*Method that contains the appropriate algorithms for each motor in the mecanum drive train.
     This method is called in our autonomous and teleop programs to reduce duplicate code and inconsistencies
     The algrotithms are adpated from our TeleOp. The arguments represent the joystick values that would be put in if ues in TeleOp.
    */

   /*Argument Breakdown:
     dirX - Represents left joystick X value
     dirY - Represents left joystick Y value
     pivot - Represents right joystick X value
    */
   public double[] setPower(double dirX, double dirY, double pivot)
    {
        //Array is used to store motors so they can be easily accessed in the method call based on the return value
        double[] motorPowers = new double[4];
        motorPowers[0] = (dirY + dirX) + pivot;//robot.motorRF.setPower(speed*((-gamepad1.left_stick_y - gamepad1.left_stick_x) - (zScale * gamepad1.right_stick_x)));
        motorPowers[1] = -(dirX - dirY) + pivot;//robot.motorRB.setPower(speed*(-(-gamepad1.left_stick_x + gamepad1.left_stick_y) - (zScale * gamepad1.right_stick_x)));
        motorPowers[2] = (dirY + dirX) - pivot;//robot.motorLB.setPower(speed*((gamepad1.left_stick_y + gamepad1.left_stick_x) - (zScale * gamepad1.right_stick_x)));
        motorPowers[3] = (-dirX + dirY) - pivot;//robot.motorLF.setPower(speed*((-gamepad1.left_stick_x + gamepad1.left_stick_y)) - (zScale * gamepad1.right_stick_x));

        //References
            //motorPowers[0] = motorRF
            //motorPowers[1] = motorRB
            //motorPowers[2] = motorLB
            //motorPowers[3] = motorLF

        return motorPowers;
    }


    //Method that drives the robot via encoder target values and what the current encoder value of the motors are

    /*
    Argument Breakdown:
    encoderDelta - Desired total change of the starting encoder value. How far the robot will go via encoder count readings
    driveStyle - Desired direction the robot will drived. Uses enumeration declared at the top of the class
    motorPower - Desired motor power the drive motors will run at
    motors - Array that contains the drive motors. This is passed in so we can use the motors from an outside class (OpMode) in this class
     */
    public boolean encoderDrive(int encoderDelta, driveStyle drive, double motorPower, DcMotor[] motors)
    {
        //ElapsedTime runtime = new ElapsedTime();

        //Comments in FORWARD also apply for all the other cases in this method

        //Switch statement used to handle which driveStyle enumeration was selected
        switch(drive)
        {
            //If desired drive direction was forward
            case FORWARD:
            {
                //Declares a sets a variable for the starting encoder value on a specific motor
                double encoderReadingLB = motors[2].getCurrentPosition();
                //Calculates desired encoder value by adding/subtracting the reading taken above by the desired encoder delta
                double target = (encoderReadingLB - encoderDelta);

                //Method declaration that will set the correct motor powers to move the robot the desired direction (based on which case you are in) with desired motor power
                forward(motorPower, motors);

                /*
                Loop that haults the code from progressing till the desired encoder count is met.
                This desired encoder value could either be positive or negative, so the appropriate logic is applied.
                */
                while (motors[2].getCurrentPosition() >= target)
                {

                }




                break;


            }

            case BACKWARD:
            {
                double encoderReadingLB = motors[2].getCurrentPosition();
                double target = (encoderDelta + encoderReadingLB);
                backward(motorPower, motors);

                while (motors[2].getCurrentPosition() <= target)
                {

                }


                break;
            }

            case STRAFE_LEFT:
            {
                double encoderReadingLB = motors[2].getCurrentPosition();
                double target = (encoderReadingLB - encoderDelta);
                strafeLeft(motorPower, motors);

                while (motors[2].getCurrentPosition() >= target)
                {

                }


                break;
            }

            case STRAFE_RIGHT:
            {
                double encoderReadingLB = motors[2].getCurrentPosition();
                double target = (encoderReadingLB + encoderDelta);
                strafeRight(motorPower, motors);

                while (motors[1].getCurrentPosition() <= target)
                {

                }


                break;
            }

            case FORWARD_LEFT:
            {
                double encoderReadingLB = motors[2].getCurrentPosition();
                double target = (encoderReadingLB - encoderDelta);
                forwardLeft(motorPower, motors);
                while (motors[2].getCurrentPosition() >= target)
                {

                }


                break;
            }

            case FORWARD_RIGHT:
            {
                double encoderReadingRB = motors[1].getCurrentPosition();
                double target = (encoderReadingRB + encoderDelta);
                forwardRight(motorPower, motors);

                while (motors[1].getCurrentPosition() <= target)
                {

                }


                break;
            }

            case BACKWARD_LEFT:
            {
                double encoderReadingRB = motors[1].getCurrentPosition();
                double target = (encoderDelta - encoderReadingRB);
                backwardLeft(motorPower, motors);

                while (motors[1].getCurrentPosition() >= target)
                {

                }


                break;
            }

            case BACKWARD_RIGHT:
            {
                double encoderReadingLB = motors[2].getCurrentPosition();
                double target = (encoderReadingLB + encoderDelta);
                backwardRight(motorPower, motors);

                while (motors[2].getCurrentPosition() <= target)
                {

                }


                break;
            }

            case PIVOT_LEFT:
            {
                double encoderReadingLB = motors[2].getCurrentPosition();
                double target = (encoderReadingLB + encoderDelta);
                pivotLeft(motorPower, motors);

                while (motors[2].getCurrentPosition() <= target)
                {

                }


                break;
            }

            case PIVOT_RIGHT:
            {
                double encoderReadingLB = motors[2].getCurrentPosition();
                double target = (encoderDelta - encoderReadingLB);
                pivotRight(motorPower, motors);

                while (motors[2].getCurrentPosition() >= target)
                {

                }


                break;
            }


        }

        //Stops all the motors
        motors[0].setPower(setPower(0, 0, 0)[0]);
        motors[1].setPower(setPower(0, 0, 0)[1]);
        motors[2].setPower(setPower(0, 0, 0)[2]);
        motors[3].setPower(setPower(0, 0, 0)[3]);

       //Return value to see if the method was successfully executed
       return true;
    }

    public void timeDrive(long time, double motorPower, driveStyle drive, DcMotor[] motors)
    {
        switch(drive)
        {
            case FORWARD:
                {
                    forward(motorPower, motors);

                    sleep(time);


                    break;


                }

            case BACKWARD:
            {
                backward(motorPower, motors);

                sleep(time);


                break;

            }

            case STRAFE_LEFT:
            {
                strafeLeft(motorPower, motors);

                sleep(time);


                break;
            }

            case STRAFE_RIGHT:
            {
                strafeRight(motorPower, motors);

                sleep(time);

                break;
            }

            case FORWARD_LEFT:
            {
                forwardLeft(motorPower, motors);

                sleep(time);

                break;
            }

            case FORWARD_RIGHT:
            {
                forwardRight(motorPower, motors);


                sleep(time);

                break;
            }

            case BACKWARD_LEFT:
            {
                backwardLeft(motorPower, motors);

                sleep(time);

                break;
            }

            case BACKWARD_RIGHT:
            {
                backwardRight(motorPower, motors);

                sleep(time);

                break;
            }

            case PIVOT_LEFT:
            {
                pivotLeft(motorPower, motors);




                sleep(time);

                break;
            }

            case PIVOT_RIGHT:
            {
                pivotRight(motorPower, motors);


                sleep(time);


                break;
            }
        }


        motors[0].setPower(setPower(-motorPower, 0, 0)[0]);
        motors[1].setPower(setPower(-motorPower, 0, 0)[1]);
        motors[2].setPower(setPower(-motorPower, 0, 0)[2]);
        motors[3].setPower(setPower(-motorPower, 0, 0)[3]);

    }
    public void OrientationDrive(double TargetOr, double motorPower, DcMotor[] motors, BNO055IMU imu) {
        Orientation angles;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double PivotDeg = 0;
        PivotDeg = (TargetOr - AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));

        if (PivotDeg > 0)
        {
            pivotLeft(motorPower, motors);

            while (TargetOr > AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
            motors[0].setPower(setPower(0, 0, 0)[0]);
            motors[1].setPower(setPower(0, 0, 0)[1]);
            motors[2].setPower(setPower(0, 0, 0)[2]);
            motors[3].setPower(setPower(0, 0, 0)[3]);
        }
        else
        {
            pivotRight(motorPower, motors);

            while (TargetOr < AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }

            motors[0].setPower(setPower(0, 0, 0)[0]);
            motors[1].setPower(setPower(0, 0, 0)[1]);
            motors[2].setPower(setPower(0, 0, 0)[2]);
            motors[3].setPower(setPower(0, 0, 0)[3]);

        }
    }


    public void forward(double motorPower, DcMotor[] motors)
    {
        motors[0].setPower(setPower(0, -motorPower, 0)[0]);
        motors[1].setPower(setPower(0, -motorPower, 0)[1]);
        motors[2].setPower(setPower(0, -motorPower, 0)[2]);
        motors[3].setPower(setPower(0, -motorPower, 0)[3]);
    }
    public void backward(double motorPower, DcMotor[] motors)
    {
        motors[0].setPower(setPower(0, motorPower, 0)[0]);
        motors[1].setPower(setPower(0, motorPower, 0)[1]);
        motors[2].setPower(setPower(0, motorPower, 0)[2]);
        motors[3].setPower(setPower(0, motorPower, 0)[3]);
    }
    public void strafeLeft(double motorPower, DcMotor[] motors)
    {
        motors[0].setPower(setPower(-motorPower, 0, 0)[0]);
        motors[1].setPower(setPower(-motorPower, 0, 0)[1]);
        motors[2].setPower(setPower(-motorPower, 0, 0)[2]);
        motors[3].setPower(setPower(-motorPower, 0, 0)[3]);
    }
    public void strafeRight(double motorPower, DcMotor[] motors)
    {
        motors[0].setPower(setPower(motorPower, 0, 0)[0]);
        motors[1].setPower(setPower(motorPower, 0, 0)[1]);
        motors[2].setPower(setPower(motorPower, 0, 0)[2]);
        motors[3].setPower(setPower(motorPower, 0, 0)[3]);
    }
    public void forwardLeft(double motorPower, DcMotor[] motors)
    {
        motors[0].setPower(setPower(-motorPower, -motorPower, 0)[0]);
        motors[1].setPower(setPower(-motorPower, -motorPower, 0)[1]);
        motors[2].setPower(setPower(-motorPower, -motorPower, 0)[2]);
        motors[3].setPower(setPower(-motorPower, -motorPower, 0)[3]);
    }
    public void forwardRight(double motorPower, DcMotor[] motors)
    {
        motors[0].setPower(setPower(motorPower, -motorPower, 0)[0]);
        motors[1].setPower(setPower(motorPower, -motorPower, 0)[1]);
        motors[2].setPower(setPower(motorPower, -motorPower, 0)[2]);
        motors[3].setPower(setPower(motorPower, -motorPower, 0)[3]);
    }
    public void backwardLeft(double motorPower, DcMotor[] motors)
    {
        motors[0].setPower(setPower(-motorPower, motorPower, 0)[0]);
        motors[1].setPower(setPower(-motorPower, motorPower, 0)[1]);
        motors[2].setPower(setPower(-motorPower, motorPower, 0)[2]);
        motors[3].setPower(setPower(-motorPower, motorPower, 0)[3]);
    }
    public void backwardRight(double motorPower, DcMotor[] motors)
    {
        motors[0].setPower(setPower(motorPower, motorPower, 0)[0]);
        motors[1].setPower(setPower(motorPower, motorPower, 0)[1]);
        motors[2].setPower(setPower(motorPower, motorPower, 0)[2]);
        motors[3].setPower(setPower(motorPower, motorPower, 0)[3]);
    }
    public void pivotLeft(double motorPower, DcMotor[] motors)
    {
        motors[0].setPower(setPower(0, 0, -motorPower)[0]);
        motors[1].setPower(setPower(0, 0, -motorPower)[1]);
        motors[2].setPower(setPower(0, 0, -motorPower)[2]);
        motors[3].setPower(setPower(0, 0, -motorPower)[3]);

    }
    public void pivotRight(double motorPower, DcMotor[] motors)
    {
        motors[0].setPower(setPower(0, 0, motorPower)[0]);
        motors[1].setPower(setPower(0, 0, motorPower)[1]);
        motors[2].setPower(setPower(0, 0, motorPower)[2]);
        motors[3].setPower(setPower(0, 0, motorPower)[3]);
    }

       /*public void methodShareDriveForward(double motorPower, double methodTime, DcMotor[] motors)
       {
           motors[0].setPower(setPower(0, -motorPower, 0)[0]);
           motors[1].setPower(setPower(0, -motorPower, 0)[1]);
           motors[2].setPower(setPower(0, -motorPower, 0)[2]);
           motors[3].setPower(setPower(0, -motorPower, 0)[3]);
           time++;
           sleep(1);
           if (time >= methodTime)
           {
               motors[0].setPower(setPower(0, 0, 0)[0]);
               motors[1].setPower(setPower(0, 0, 0)[1]);
               motors[2].setPower(setPower(0, 0, 0)[2]);
               motors[3].setPower(setPower(0, 0, 0)[3]);
               isMethodShareForwardFinished = true;
           }
       }

        public void methodShareModPower(double motorPower, DcMotor[] motors)
        {
            motors[0].setPower(setPower(0, -motorPower, 0)[0]);
            motors[1].setPower(setPower(0, -motorPower, 0)[1]);
            motors[2].setPower(setPower(0, -motorPower, 0)[2]);
            motors[3].setPower(setPower(0, -motorPower, 0)[3]);
        }

        public void methodShareEncoderTest(double motorPower, double methodEncoder, DcMotor[] motors)
        {
            encoder = motors[1].getCurrentPosition();
            motors[0].setPower(setPower(0, -motorPower, 0)[0]);
            motors[1].setPower(setPower(0, -motorPower, 0)[1]);
            motors[2].setPower(setPower(0, -motorPower, 0)[2]);
            motors[3].setPower(setPower(0, -motorPower, 0)[3]);
            if(encoder >= methodEncoder + startingEncoder)
            {
                motors[0].setPower(setPower(0, 0, 0)[0]);
                motors[1].setPower(setPower(0, 0, 0)[1]);
                motors[2].setPower(setPower(0, 0, 0)[2]);
                motors[3].setPower(setPower(0, 0, 0)[3]);
                isMethodShareEncoderTestFinished = true;
            }
        }
        */



    @Override
    public void runOpMode()
    {}


}
