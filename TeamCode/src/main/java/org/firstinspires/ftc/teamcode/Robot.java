package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {


    //---DRIVING---//
    //Driving
    public static DcMotorEx backLeft;
    public static DcMotorEx backRight;
    public static DcMotorEx frontLeft;
    public static DcMotorEx frontRight;
    public static double wheelTicksPerRevolution = 383.6;  // for 13.7:1 gearboxes
    public static double wheelDiameter = 3.77;
    public static double ticksPerInch = (wheelTicksPerRevolution) / (wheelDiameter * Math.PI);

    public static double deadStickZone = 0.01;
    public static double wheelPowerMinToMove = 0.05;

    // Color Sensing

    public static ColorSensor midLeft;
    public static ColorSensor midRight;

    public static ColorSensor outsideLeft;
    public static ColorSensor outsideRight;

    public static double redThreshold = 60;
    public static double onLineThreshold = 50;



    //---MISC---//
    public static boolean rightTurnAvailable() {
        return Robot.outsideRight.red() >= 90;

    }
    public static boolean leftTurnAvailable() {
        return Robot.outsideLeft.red() >= 90;

    }

    public static void signalRightTurn() {
        telemetry.addLine("Right Turn Available");
        Gamepad.RumbleEffect rumbleRight;
        rumbleRight = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec

                .build();
        gamepad1.runRumbleEffect(rumbleRight);
    }
    public static void signalLeftTurn() {
        telemetry.addLine("Left Turn Available");
        Gamepad.RumbleEffect rumbleLeft;
        rumbleLeft = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 0.0, 500)  //  Rumble left motor 100% for 500 mSec

                .build();
        gamepad1.runRumbleEffect(rumbleLeft);
    }

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

        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //Color sensors

        midLeft = hardwareMap.get(ColorSensor.class, "midLeft");
        midRight = hardwareMap.get(ColorSensor.class, "midRight");

        outsideLeft = hardwareMap.get(ColorSensor.class, "outsideLeft");
        outsideRight = hardwareMap.get(ColorSensor.class, "outsideRight");

    }

}