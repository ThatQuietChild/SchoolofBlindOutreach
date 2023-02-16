package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {


    //---DRIVING---//
    //Driving
    public static DcMotorEx backLeft;
    public static DcMotorEx backRight;
    public static DcMotorEx frontLeft;
    public static DcMotorEx frontRight;
    public static double wheelCountsPerRevolution = 537.6;
    public static double wheelDiameter = 3.77;
    public static double ticksPerInch = (wheelCountsPerRevolution) /
            (wheelDiameter * Math.PI);

    public static double deadStickZone = 0.01;
    public static double wheelPowerMinToMove = 0.05;



    public static ColorSensor midLeft;
    public static ColorSensor midRight;

    public static ColorSensor outsideLeft;
    public static ColorSensor outsideRight;

    //---MISC---//

    //Hardware Map
    public HardwareMap hardwareMap;



    public Robot(HardwareMap robot_hardwareMap) {
        //Set Hardware map
        hardwareMap = robot_hardwareMap;

        //---Driving---//
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");


        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Color sensors

        midLeft = hardwareMap.get(ColorSensor.class, "midLeft");
        midRight = hardwareMap.get(ColorSensor.class, "midRight");

        outsideLeft = hardwareMap.get(ColorSensor.class, "outsideLeft");
        outsideRight = hardwareMap.get(ColorSensor.class, "outsideRight");

    }
}