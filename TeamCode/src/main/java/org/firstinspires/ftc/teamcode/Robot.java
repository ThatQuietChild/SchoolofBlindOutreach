package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Robot {

    //Driving
    public static DcMotorEx backLeft;
    public static DcMotorEx backRight;
    public static DcMotorEx frontLeft;
    public static DcMotorEx frontRight;

    public static double wheelTicksPerRevolution = 383.6;  // for 13.7:1 gearboxes
    public static double wheelDiameter = 3.77;
    public static double ticksPerInch = (wheelTicksPerRevolution) / (wheelDiameter * Math.PI);
    public static double fullSpeed = 0.17;
    public static double slowSpeed = 0.09;

    // Color Sensing
    public static ColorSensor midLeft;
    public static ColorSensor midRight;

    public static ColorSensor outsideLeft;
    public static ColorSensor outsideRight;

    public static double redThreshold = 75;

    public static Gamepad.RumbleEffect rumbleRight = new Gamepad.RumbleEffect.Builder()
            .addStep(0.0, 1.0, 150)  //  Rumble right motor 100% for 500 mSec
            .addStep(0.0, 0, 100)  //  Rumble right motor 100% for 500 mSec
            .addStep(0.0, 1.0, 150)  //  Rumble right motor 100% for 500 mSec
            .addStep(0.0, 0, 100)  //  Rumble right motor 100% for 500 mSec
            .addStep(0.0, 1.0, 150)  //  Rumble right motor 100% for 500 mSec
            .addStep(0.0, 0, 100)  //  Rumble right motor 100% for 500 mSec
            .build();

    public static Gamepad.RumbleEffect rumbleLeft = new Gamepad.RumbleEffect.Builder()
            .addStep(1.0, 0.0, 200)  //  Rumble left motor 100% for 500 mSec
            .addStep(0, 0.0, 100)  //  Rumble left motor 100% for 500 mSec
            .addStep(1.0, 0.0, 200)  //  Rumble left motor 100% for 500 mSec
            .addStep(0, 0.0, 100)  //  Rumble left motor 100% for 500 mSec
            .build();

    //Hardware Map
    public HardwareMap hardwareMap;


    public Robot(HardwareMap robot_hardwareMap) {
        //Set Hardware map
        hardwareMap = robot_hardwareMap;

        //---Driving---//
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //Color sensors

        midLeft = hardwareMap.get(ColorSensor.class, "midLeft");
        midRight = hardwareMap.get(ColorSensor.class, "midRight");

        outsideLeft = hardwareMap.get(ColorSensor.class, "outsideLeft");
        outsideRight = hardwareMap.get(ColorSensor.class, "outsideRight");

    }
}