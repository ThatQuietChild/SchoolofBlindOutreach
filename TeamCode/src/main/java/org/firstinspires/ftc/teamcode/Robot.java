package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    // Color Sensing
    public static ColorSensor midLeft;
    public static ColorSensor midRight;

    public static ColorSensor outsideLeft;
    public static ColorSensor outsideRight;

    public static double redThreshold = 60;


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

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Color sensors

        midLeft = hardwareMap.get(ColorSensor.class, "midLeft");
        midRight = hardwareMap.get(ColorSensor.class, "midRight");

        outsideLeft = hardwareMap.get(ColorSensor.class, "outsideLeft");
        outsideRight = hardwareMap.get(ColorSensor.class, "outsideRight");
    }
}