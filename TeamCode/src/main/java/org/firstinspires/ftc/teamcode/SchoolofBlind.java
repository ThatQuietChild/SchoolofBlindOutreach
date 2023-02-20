package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp

public class SchoolofBlind extends LinearOpMode {

    BNO055IMU imu;
    Orientation angles;

    // global variables for getActualHeading.  Do not define anywhere else
    double actualHeading = 0;
    boolean goRight;
    double degreesOff;

    @Override
    public void runOpMode() {
        org.firstinspires.ftc.teamcode.Robot robot = new org.firstinspires.ftc.teamcode.Robot(hardwareMap);

        initIMU();

        waitForStart();
        Gamepad.RumbleEffect rumbleLeft;
        rumbleLeft = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 0.0, 500)  //  Rumble left motor 100% for 500 mSec

                .build();
        Gamepad.RumbleEffect rumbleRight;
        rumbleRight = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec

                .build();
        double defaultWheelPower = .25;
        double currentHeading = 0;

        while (opModeIsActive()) {

            // Get gamepad values
            // dpad up and down are forward and backward
            
            boolean pressingForward = gamepad1.dpad_up;
            boolean pressingReverse = gamepad1.dpad_down;
            
            // dpad left and right are for turning 90 left/right

            boolean pressingRightTurn = gamepad1.dpad_right;
            boolean pressingLeftTurn = gamepad1.dpad_left;

            double wheelPower;
            boolean prevDirectionForward = false;
            double frontLeftPower;
            double backLeftPower;
            double fontRightPower;
            double backRightPower;

            // right turn
            if (pressingRightTurn && rightTurnAvailable()) {
                currentHeading = turn(currentHeading, true);
            }

            // left turn
            if (pressingLeftTurn && leftTurnAvailable()) {
                currentHeading = turn(currentHeading, false);
            }

            // signal right available
            if (rightTurnAvailable()) {
                signalRightTurn();
            }

            // signal left available
            if (leftTurnAvailable()) {
                signalLeftTurn();
            }
            
            // Driving code
            
            if (pressingForward && robotIsOnTheLine()) {
                wheelPower = defaultWheelPower;
                prevDirectionForward = true;
            } else {
                if (pressingReverse && robotIsOnTheLine()) {
                    wheelPower = -defaultWheelPower;
                    prevDirectionForward = false;
                } else {
                    wheelPower = 0;
                }
            }

            if (!robotIsOnTheLine()) {    // special case to get it back on the line after it goes off.  Dangerous though.  Robot would drive forever until it finds red
                if (prevDirectionForward) {
                    wheelPower = -defaultWheelPower;
                } else {
                    wheelPower = defaultWheelPower;
                }
            }

            double headingAdjustPower = 0;
            double colorAdjustPower = 0;
            if (wheelPower != 0) {
                colorAdjustPower = adjustForColor();
                headingAdjustPower = headingAdjustment(currentHeading);
            }

            frontLeftPower = wheelPower - headingAdjustPower + colorAdjustPower;
            backLeftPower = wheelPower - headingAdjustPower - colorAdjustPower;
            fontRightPower = wheelPower + headingAdjustPower - colorAdjustPower;
            backRightPower = wheelPower + headingAdjustPower + colorAdjustPower;

            robot.frontLeft.setPower(frontLeftPower);
            robot.backLeft.setPower(backLeftPower);
            robot.frontRight.setPower(fontRightPower);
            robot.backRight.setPower(backRightPower);

            telemetry.addData("mid left", Robot.midLeft.red());
            telemetry.addData("mid right", Robot.midRight.red());
            telemetry.addData("outside left", Robot.outsideLeft.red());
            telemetry.addData("outside right", Robot.outsideRight.red());
            telemetry.addData("currentHeading", currentHeading);
            telemetry.addData("actualHeading", actualHeading);

            telemetry.update();
        }
    }
    public void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void getActualHeading(double targetHeading) {        // Also sets degreesOff and goRight
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        actualHeading = (360 + angles.firstAngle) % 360;        // This always gets you a number between 0 and 359.99
        goRight = targetHeading > actualHeading;                // Simply, if it's a higher number, we want to go clockwise or right
        degreesOff = Math.abs(targetHeading - actualHeading);
        if (degreesOff > 180) {                                 // But if turning the other way is shorter, switch it
            goRight = !goRight;
            degreesOff = 360 - degreesOff;
        }
    }

    public double turn(double prevHeading, boolean turnRight) {  // Only options are to turn right +90 or left -90

        double turnPower = .25;
        double turnReverse;
        double targetHeading;

        if (turnRight) {
            targetHeading = prevHeading - 90;
        } else {
            targetHeading = prevHeading + 90;
        }
        targetHeading = (360 + targetHeading) % 360;
        
        getActualHeading(targetHeading);

        while (degreesOff > .3 && opModeIsActive()) {

            if (goRight) {
                turnReverse = 1; // turn right
            } else {
                turnReverse = -1; // turn left
            }

            // think about using a calculation here for turnPower.

            Robot.frontLeft.setPower(turnPower * turnReverse);
            Robot.backLeft.setPower(turnPower * turnReverse);
            Robot.frontRight.setPower(-turnPower * turnReverse);
            Robot.backRight.setPower(-turnPower * turnReverse);

            getActualHeading(targetHeading);

        }

        Robot.frontLeft.setPower(0);
        Robot.backLeft.setPower(0);
        Robot.frontRight.setPower(0);
        Robot.backRight.setPower(0);

        return targetHeading;
    }

    public double headingAdjustment(double targetHeading) {
        double adjustment;
        double speedMinimum = 2;
        double speedModifier = 8;
        double graphShift = 0;
        double curvePower = 2;

        getActualHeading(targetHeading);

        if (degreesOff < .3) {
            adjustment = 0;
        } else {
            adjustment = (Math.pow((degreesOff + graphShift) / speedModifier, curvePower) + speedMinimum) / 100;
        }

        if (goRight) {
            adjustment = -adjustment;
        }
        return adjustment;
    }

    public double adjustForColor() {
        double leftColor;
        double rightColor;


        double redDivisor = 1000;

        double outputValue = 0;

        leftColor = Robot.midLeft.red();
        rightColor = Robot.midRight.red();

        if (leftColor > Robot.redThreshold || rightColor > Robot.redThreshold) {
            outputValue = (rightColor - leftColor) / redDivisor;
        }

        return outputValue;
    }
    public boolean robotIsOnTheLine(){
        if(Robot.midLeft.red() < Robot.redThreshold && Robot.midRight.red() < Robot.redThreshold){
            return false;
        }
        else {
            return true;
        }
    }

    public boolean rightTurnAvailable() {
        return Robot.outsideRight.red() >= 90;

    }
    public boolean leftTurnAvailable () {
        return Robot.outsideLeft.red() >= 90;

    }

    public void signalRightTurn() {
        telemetry.addLine("Right Turn Available");
        Gamepad.RumbleEffect rumbleRight;
        rumbleRight = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec

                .build();
        gamepad1.runRumbleEffect(rumbleRight);
    }

    public void signalLeftTurn() {
        telemetry.addLine("Left Turn Available");
        Gamepad.RumbleEffect rumbleLeft;
        rumbleLeft = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 0.0, 500)  //  Rumble left motor 100% for 500 mSec

                .build();
        gamepad1.runRumbleEffect(rumbleLeft);
    }

    public void drive(double speed, double inches) {
        int ticksToMove = (int) (inches * Robot.ticksPerInch);
        int savePosition = Robot.frontLeft.getCurrentPosition();
        Robot.frontLeft.setTargetPosition(savePosition + ticksToMove);
        Robot.backLeft.setTargetPosition(Robot.backLeft.getCurrentPosition() + ticksToMove);
        Robot.frontRight.setTargetPosition(Robot.frontRight.getCurrentPosition() + ticksToMove);
        Robot.backRight.setTargetPosition(Robot.backLeft.getCurrentPosition() + ticksToMove);

        Robot.frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        Robot.backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        Robot.frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        Robot.backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        Robot.frontLeft.setPower(speed);
        Robot.backLeft.setPower(speed);
        Robot.frontRight.setPower(speed);
        Robot.backRight.setPower(speed);

        while (Robot.frontLeft.isBusy() && opModeIsActive()) {
            telemetry.addData("Inches driven", (Robot.frontLeft.getCurrentPosition() - savePosition) / Robot.ticksPerInch);
            telemetry.update();
        }

        Robot.frontLeft.setPower(0);
        Robot.backLeft.setPower(0);
        Robot.frontRight.setPower(0);
        Robot.backRight.setPower(0);

        Robot.frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Robot.backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Robot.frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Robot.backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
}