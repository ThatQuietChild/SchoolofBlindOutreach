package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "Library of Blind")

public class SchoolofBlind extends LinearOpMode {

    BNO055IMU imu;
    Orientation angles;

    // global variables for getActualHeading.  Do not define anywhere else
    public double actualHeading = 0;
    public boolean goLeft;
    public double degreesOff;
    public double currentHeading = 0;

    ElapsedTime gameTimer = new ElapsedTime();
    public double prevLTSignalTime = 0;
    public double prevRTSignalTIme = 0;
    public double prevTreasureSignalTIme = 0;
    public double prevSignalHeading = -1;

    @Override
    public void runOpMode() {
        new Robot(hardwareMap);
        initIMU();

        waitForStart();

        double defaultWheelPower = Robot.fullSpeed;
        double wheelPower = 0;
        boolean prevDirectionForward = false;
        boolean reachedEndOfTape = true;
        double frontLeftPower;
        double backLeftPower;
        double frontRightPower;
        double backRightPower;

        gameTimer.reset();


        while (opModeIsActive()) {

            // Get gamepad values
            // dpad up and down are forward and backward
            
            boolean pressingForward = gamepad1.left_stick_y < 0;
            boolean pressingReverse = gamepad1.left_stick_y > 0;

            defaultWheelPower = Math.abs(Robot.fullSpeed * gamepad1.left_stick_y);
            
            // dpad left and right are for turning 90 left/right

            boolean pressingRightTurn = gamepad1.b;
            boolean pressingLeftTurn = gamepad1.x;

            // right turn
            if (pressingRightTurn) {
                    if (rightTurnAvailable()){
                        currentHeading = turn(currentHeading, true);
                    }
                }


            // left turn
            if (pressingLeftTurn) {
                if(leftTurnAvailable()){
                    currentHeading = turn(currentHeading, false);
                }
            }

            // signal right available
            if (rightTurnAvailable()) {
                signalRightTurn();
            }
            // signal left available
            if (leftTurnAvailable()) {
                signalLeftTurn();  // might need to put a timer in this and only signal every second
            }


            // Driving code

            if (pressingForward && robotIsOnTheLine()) {
                wheelPower = defaultWheelPower;
                prevDirectionForward = true;
                reachedEndOfTape = false;
            } else {
                if (pressingReverse && robotIsOnTheLine()) {
                    wheelPower = -defaultWheelPower;
                    prevDirectionForward = false;
                    reachedEndOfTape = false;
                } else {
                    wheelPower = 0;
                }
            }

            if (!robotIsOnTheLine() && !reachedEndOfTape) {    // special case to get it back on the line after it goes off.
                signalEndOfTape();
                reachedEndOfTape = true;  // this is so we only run this once
                if (prevDirectionForward) {
                    drive(-Robot.fullSpeed, 4);
                } else {
                    drive(Robot.fullSpeed, 4);
                }
            }

            double headingAdjustPower = 0;  // default to zero
            double colorAdjustPower = 0;    // default to zero

            if (wheelPower != 0) {          // if driving, then do adjustments
                colorAdjustPower = adjustForColor();
                headingAdjustPower = headingAdjustment(currentHeading);
            }

            frontLeftPower = wheelPower - headingAdjustPower + colorAdjustPower;
            backLeftPower = wheelPower - headingAdjustPower - colorAdjustPower;
            frontRightPower = wheelPower + headingAdjustPower - colorAdjustPower;
            backRightPower = wheelPower + headingAdjustPower + colorAdjustPower;

            Robot.frontLeft.setPower(frontLeftPower);
            Robot.backLeft.setPower(backLeftPower);
            Robot.frontRight.setPower(frontRightPower);
            Robot.backRight.setPower(backRightPower);

            telemetry.addData("mid left", Robot.midLeft.red());
            telemetry.addData("mid right", Robot.midRight.red());
            telemetry.addData("outside left", Robot.outsideLeft.red());
            telemetry.addData("outside right", Robot.outsideRight.red());
            telemetry.addData("wheelPower", wheelPower);
            telemetry.addData("colorAdjustPower", colorAdjustPower);
            telemetry.addData("headingAdjustPower", headingAdjustPower);
            telemetry.addData("currentHeading", currentHeading);
            telemetry.addData("actualHeading", actualHeading);
            telemetry.addData("outside right blue", Robot.outsideRight.blue());
            telemetry.addData("outside left blue", Robot.outsideLeft.blue());
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

    public void getActualHeading(double targetHeading) {        // Also sets degreesOff and goLeft
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        actualHeading = (360 + angles.firstAngle) % 360;        // This always gets you a number between 0 and 359.99
        goLeft = targetHeading > actualHeading;                // Simply, if it's a higher number, we want to go clockwise or right
        degreesOff = Math.abs(targetHeading - actualHeading);
        if (degreesOff > 180) {                                 // But if turning the other way is shorter, switch it
            goLeft = !goLeft;
            degreesOff = 360 - degreesOff;
        }
    }

    public double turn(double prevHeading, boolean turnRight) {  // Only options are to turn right +90 or left -90

        int northID   = hardwareMap.appContext.getResources().getIdentifier("north",   "raw", hardwareMap.appContext.getPackageName());
        int westID   = hardwareMap.appContext.getResources().getIdentifier("west",   "raw", hardwareMap.appContext.getPackageName());
        int southID   = hardwareMap.appContext.getResources().getIdentifier("south",   "raw", hardwareMap.appContext.getPackageName());
        int eastID   = hardwareMap.appContext.getResources().getIdentifier("east",   "raw", hardwareMap.appContext.getPackageName());

        double turnPower = .25;
        double turnReverse;
        double targetHeading;
        ElapsedTime turnTimer = new ElapsedTime();
        turnTimer.reset();

        if (turnRight) {
            targetHeading = prevHeading - 90;
        } else {
            targetHeading = prevHeading + 90;
        }
        targetHeading = (360 + targetHeading) % 360;
        
        getActualHeading(targetHeading);

        double savePrevTime = turnTimer.milliseconds();
        double savePrevDegrees = actualHeading;
        double degreesPerSecond = 0;

        while (degreesOff > .3 || degreesPerSecond > 1.5 && opModeIsActive()) {

            if (goLeft) {
                turnReverse = 1; // turn left
            } else {
                turnReverse = -1; // turn right
            }

            // think about using a calculation here for turnPower.
            if (degreesOff < 10) {
                turnPower = .10;
            }

            Robot.frontLeft.setPower(-turnPower * turnReverse);
            Robot.backLeft.setPower(-turnPower * turnReverse);
            Robot.frontRight.setPower(turnPower * turnReverse);
            Robot.backRight.setPower(turnPower * turnReverse);

            getActualHeading(targetHeading);
            degreesPerSecond = Math.abs(actualHeading - savePrevDegrees) / (turnTimer.milliseconds() - savePrevTime) / 1000;
            savePrevTime = turnTimer.milliseconds();
            savePrevDegrees = actualHeading;
        }

        Robot.frontLeft.setPower(0);
        Robot.backLeft.setPower(0);
        Robot.frontRight.setPower(0);
        Robot.backRight.setPower(0);

        if (targetHeading == 0) {
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, northID);
        } else if (targetHeading == 90) {
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, westID);
        } else if (targetHeading == 180) {
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, southID);
        } else if (targetHeading == 270) {
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, eastID);
        }

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

        if (!goLeft) {
            adjustment = -adjustment;  // positive values mean goLeft (i.e. heading is off target to the right), negative value is go right
        }
        return adjustment;
    }

    public double adjustForColor() {
        double leftColor;
        double rightColor;

        double redDivisor = 5000;
        double maxRed = 250;
        double outputValue = 0;

        leftColor = Robot.midLeft.red();
        rightColor = Robot.midRight.red();

        if (leftColor > maxRed && rightColor > maxRed) {  // if both are on the line close enough, do nothing
            outputValue = 0;
        } else {
            if (leftColor > Robot.redThreshold || rightColor > Robot.redThreshold) {
                outputValue = (rightColor - leftColor) / redDivisor;
            }
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
        return Robot.outsideRight.red() >= 40;
    }

    public boolean leftTurnAvailable () {
        return Robot.outsideLeft.red() >= 40;
    }

    public void signalRightTurn() {
        int rightTurnID   = hardwareMap.appContext.getResources().getIdentifier("rightturn",   "raw", hardwareMap.appContext.getPackageName());
        telemetry.addLine("Right Turn Available");
        if ((gameTimer.seconds() - prevLTSignalTime) > 4 /*|| prevSignalHeading != currentHeading*/) {  // only signal if 2 seconds has passed or we have turned
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, rightTurnID);
            gamepad1.runRumbleEffect(Robot.rumbleRight);
            prevLTSignalTime = gameTimer.seconds();
            prevSignalHeading = currentHeading;
        }
    }

    public void signalLeftTurn() {
        int leftTurnID = hardwareMap.appContext.getResources().getIdentifier("leftturn", "raw", hardwareMap.appContext.getPackageName());
        telemetry.addLine("Left Turn Available");
        if ((gameTimer.seconds() - prevRTSignalTIme) > 4 /*|| prevSignalHeading != currentHeading*/) {  // only signal if 2 seconds has passed or we have turned
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, leftTurnID);
            gamepad1.runRumbleEffect(Robot.rumbleLeft);
            prevLTSignalTime = gameTimer.seconds();
            prevSignalHeading = currentHeading;
        }
    }

    public void signalTreasure() {
        int treasureID   = hardwareMap.appContext.getResources().getIdentifier("treasure",   "raw", hardwareMap.appContext.getPackageName());
        telemetry.addLine("Treasure!!!!!!!");
        if ((gameTimer.seconds() - prevTreasureSignalTIme) > 2) {  // only signal if 2 seconds has passed or we have turned
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, treasureID);
            //gamepad1.runRumbleEffect(Robot.rumbleLeft);
            prevTreasureSignalTIme = gameTimer.seconds();
        }
    }

    public void signalEndOfTape() {
        telemetry.addLine("EndOfTape");
        gamepad1.runRumbleEffect(Robot.rumbleRight);
    }

    public void driveToPosition(int frontLeftPos, int backLeftPos, int frontRightPos, int backRightPos) {

        Robot.frontLeft.setTargetPosition(frontLeftPos);
        Robot.backLeft.setTargetPosition(backLeftPos);
        Robot.frontRight.setTargetPosition(frontRightPos);
        Robot.backRight.setTargetPosition(backRightPos);

        Robot.frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        Robot.backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        Robot.frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        Robot.backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        double multiplier = 1;
        if (frontLeftPos < Robot.frontLeft.getCurrentPosition()){
            multiplier = -1;
        }
        else{
            multiplier = 1;
        }
        Robot.frontLeft.setPower(Robot.fullSpeed * multiplier);
        Robot.backLeft.setPower(Robot.fullSpeed * multiplier);
        Robot.frontRight.setPower(Robot.fullSpeed * multiplier);
        Robot.backRight.setPower(Robot.fullSpeed * multiplier);


        while (Robot.frontLeft.isBusy() && opModeIsActive()) {
            telemetry.addData("Inches driven", (Robot.frontLeft.getCurrentPosition()) / Robot.ticksPerInch);
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

    public void drive(double speed, double inches) {

        int ticksToMove = (int) (inches * Robot.ticksPerInch);
        if (speed < 0) {
            ticksToMove = ticksToMove * -1;
        }

        int savePosition = Robot.frontLeft.getCurrentPosition();
        Robot.frontLeft.setTargetPosition(savePosition + ticksToMove);
        Robot.backLeft.setTargetPosition(Robot.backLeft.getCurrentPosition() + ticksToMove);
        Robot.frontRight.setTargetPosition(Robot.frontRight.getCurrentPosition() + ticksToMove);
        Robot.backRight.setTargetPosition(Robot.backRight.getCurrentPosition() + ticksToMove);

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

    public double position(){
        double currentPos = (Robot.frontLeft.getCurrentPosition()+ Robot.backLeft.getCurrentPosition() + Robot.frontRight.getCurrentPosition() + Robot.backRight.getCurrentPosition())/Robot.ticksPerInch;
        return currentPos;
    }


}