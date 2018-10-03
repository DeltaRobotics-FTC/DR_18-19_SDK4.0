package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.abs;


/**
 * Created by User on 10/7/2017.
 */


enum driveStyle
{
    FORWARD, BACKWARD, STRAFE_LEFT, STRAFE_RIGHT, FORWARD_RIGHT, FORWARD_LEFT, BACKWARD_RIGHT, BACKWARD_LEFT, PIVOT_RIGHT, PIVOT_LEFT
}
public class Drive extends LinearOpMode
{

   public static double drivePower = 0.75;
    public static double strafePower = 0.85;
    public static double pivotPower = 0.6;
    public static boolean isMethodShareForwardFinished = false;
    public static boolean isMethodShareEncoderTestFinished = false;
    public static double time = 0;
    public static double encoder = 0;
    public static double startingEncoder = 0;


    public double[] setPower(double dirX, double dirY, double pivot)
    {
        double[] motorPowers = new double[4];
        motorPowers[0] = (-dirY - dirX) - pivot;
        motorPowers[1] = -(-dirX + dirY) - pivot;
        motorPowers[2] = -(-dirY - dirX) - pivot;
        motorPowers[3] = (-dirX + dirY) - pivot;
        //

        //motorPowers[0] = motorRF
        //motorPowers[1] = motorRB
        //motorPowers[2] = motorLB
        //motorPowers[3] = motorLF

        return motorPowers;
    }


    public boolean encoderDrive(int encoderDelta, driveStyle drive, double motorPower, DcMotor[] motors)
    {
        //ElapsedTime runtime = new ElapsedTime();


        switch(drive)
        {
            case FORWARD:
            {
                double encoderReadingLB = motors[2].getCurrentPosition();
                double target = (encoderReadingLB - encoderDelta);

                forward(motorPower, motors);

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
                double encoderReadingLB = motors[1].getCurrentPosition();
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

//
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

        motors[0].setPower(setPower(0, 0, 0)[0]);
        motors[1].setPower(setPower(0, 0, 0)[1]);
        motors[2].setPower(setPower(0, 0, 0)[2]);
        motors[3].setPower(setPower(0, 0, 0)[3]);

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
    public void OrientationDrive(double orientationTargetDelta, driveStyle drive, double motorPower, DcMotor[] motors, BNO055IMU imu)
    {
        Orientation angles;
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        switch(drive) {
            case PIVOT_LEFT: {
                double target = 0;
                if(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) > 0)
                {
                    target = orientationTargetDelta - AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
                }
                else
                {
                    target = orientationTargetDelta + AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
                }
                pivotLeft(motorPower, motors);

                while(target > AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle))
                {
                    angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }
                motors[0].setPower(setPower(0, 0, 0)[0]);
                motors[1].setPower(setPower(0, 0, 0)[1]);
                motors[2].setPower(setPower(0, 0, 0)[2]);
                motors[3].setPower(setPower(0, 0, 0)[3]);
                break;
            }

            case PIVOT_RIGHT: {
                double target = 0;
                if(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) <= 0)
                {
                    target = -orientationTargetDelta - AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
                }
                else
                {
                    target = -orientationTargetDelta + AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
                }

                pivotRight(motorPower, motors);

                while(target < AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle))
                {
                    angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }

                motors[0].setPower(setPower(0, 0, 0)[0]);
                motors[1].setPower(setPower(0, 0, 0)[1]);
                motors[2].setPower(setPower(0, 0, 0)[2]);
                motors[3].setPower(setPower(0, 0, 0)[3]);
                //
                break;
            }


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

       public void methodShareDriveForward(double motorPower, double methodTime, DcMotor[] motors)
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



    @Override
    public void runOpMode()
    {}


}
