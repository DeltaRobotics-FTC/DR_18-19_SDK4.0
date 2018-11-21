package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is a class that initializes all of the various devices<br/>
 * on the robot. This can also be used to set any values you want<br/>
 * when the robot initializes
 * @author Delta Robotics
 * @since 9/26/2017.
 */

public class RobotHardware
{
    /**
     * Object of Right Front motor on drive train
     */
    public DcMotor motorRF = null; //MotorRF on drive train
    /**
     * Object of Left Front motor on drive train
     */
    public DcMotor motorLF = null; //MotorLF on drive train
    /**
     * Object of Right Back motor on drive train
     */
    public DcMotor motorRB = null; //MotorRB on drive train
    /**
     * Object of Left Back motor on drive train
     */
    public DcMotor motorLB = null; //MotorLB on drive train

    public DcMotor motorLift = null; //motorLift on the the lift motor

    public DcMotor Arm = null; //Arm on the the lift motor

    public DcMotor armEXT = null; //armEXT on the the lift motor

    public Servo marker = null;

    public DigitalChannel liftLimitBottom = null;

    /**
     * Blank constructor
     */
    public RobotHardware() //Constructor
    {

    }

    /**
     * Initializes all of the devices on the<br/>
     * robot and sets any values necessary
     */
    public void init(HardwareMap ahwMap) //Code that runs when you init the hardware map
    {
        motorRF = ahwMap.dcMotor.get("motorRF"); //What to look for in config for motorRF
        motorLF = ahwMap.dcMotor.get("motorLF"); //What to look for in config for motorLF
        motorRB = ahwMap.dcMotor.get("motorRB"); //What to look for in config for motorRB
        motorLB = ahwMap.dcMotor.get("motorLB"); //What to look for in the config for motorLB
        motorLift = ahwMap.dcMotor.get("motorLift"); //What to look for in the config for motorLift
        Arm = ahwMap.dcMotor.get("Arm"); //What to look for in the config for Arm
        armEXT = ahwMap.dcMotor.get("armEXT");//What to look for in the config for armEXT


        marker = ahwMap.servo.get("marker"); // What to look for  in the config for marker

        liftLimitBottom = ahwMap.get(DigitalChannel.class, "liftLimitBottom");



        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//]
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//]
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//]
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//] Sets motors so when they have 0 power, they brake instead of coast
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//]
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//]
        armEXT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//]

        liftLimitBottom.setMode(DigitalChannel.Mode.INPUT);

        //motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        marker.setPosition(0.15);

        motorRF.setPower(0);//]
        motorLF.setPower(0);//] Stops the drive motors
        motorRB.setPower(0);//]
        motorLB.setPower(0);//]
        motorLift.setPower(0);//]
        armEXT.setPower(0);//]
        Arm.setPower(0);//]

    }
}
