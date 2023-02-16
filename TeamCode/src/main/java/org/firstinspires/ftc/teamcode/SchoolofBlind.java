package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp

public class SchoolofBlind extends LinearOpMode {

    BNO055IMU imu;
    Orientation angles;

    @Override
    public void runOpMode() {
        org.firstinspires.ftc.teamcode.Robot robot = new org.firstinspires.ftc.teamcode.Robot(hardwareMap);

        initIMU();

        waitForStart();
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
            double lfPower;
            double rfPower;
            double lrPower;
            double rrPower;

            // right turn
            if (pressingRightTurn && rightTurnAvailable()) {
                currentHeading = (currentHeading + 90) % 360;
                turnToHeading(currentHeading);
            }

            // left turn
            if (pressingLeftTurn && leftTurnAvailable()) {
                currentHeading = (360 + currentHeading - 90) % 360;
                turnToHeading(currentHeading);
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
            
            if (pressingForward) {
                wheelPower = defaultWheelPower;
            } else {
                if (pressingReverse) {
                    wheelPower = -defaultWheelPower;
                } else {
                    wheelPower = 0;
                }
            }

            double adjustment = headingAdjustment(currentHeading);

            double adjustForColorVariable = adjustForColor();

            lfPower = wheelPower - adjustment + adjustForColorVariable;
            lrPower = wheelPower - adjustment - adjustForColorVariable;

            rfPower = wheelPower + adjustment - adjustForColorVariable;
            rrPower = wheelPower + adjustment + adjustForColorVariable;

            robot.backLeft.setPower(lrPower);
            robot.backRight.setPower(rrPower);
            robot.frontLeft.setPower(lfPower);
            robot.frontRight.setPower(rfPower);

            //telemetry.addData("RL Servo: ", tapeMeasureRLServoPosition);
            //telemetry.addData("UD Servo: ", tapeMeasureUDServoPosition);
            telemetry.addData("LeftStickX", gamepad1.left_stick_x);
            telemetry.addData("LeftStickY", gamepad1.left_stick_y);
            telemetry.addData("RightStickX", gamepad1.right_stick_x);
            telemetry.addData("RightStickY", gamepad1.right_stick_y);
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

    public void turnToHeading(double targetDegrees) {

        double turnPower = .25;
        double turnReverse = 1;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentHeading = (360 + angles.firstAngle) % 360;

        double degreesOff = targetDegrees - currentHeading;

        while (Math.abs(degreesOff) > .3) {

            if (currentHeading < targetDegrees) {
                turnReverse = 1;
            } else {
                turnReverse = -1;
            }

            Robot.frontRight.setPower(-turnPower * turnReverse);
            Robot.backRight.setPower(-turnPower * turnReverse);
            Robot.frontLeft.setPower(turnPower * turnReverse);
            Robot.backLeft.setPower(turnPower * turnReverse);

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = (360 + angles.firstAngle) % 360;
            degreesOff = targetDegrees - currentHeading;
        }

        Robot.frontRight.setPower(0);
        Robot.backRight.setPower(0);
        Robot.frontLeft.setPower(0);
        Robot.backLeft.setPower(0);

    }

    public double headingAdjustment(double targetHeading) {
        double adjustment;
        double currentHeading;
        double speedMinimum = 2;
        double speedModifier = 8;
        double graphShift = 0;
        double curvePower = 2;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentHeading = (360 + angles.firstAngle) % 360;

        boolean goRight = targetHeading > currentHeading;
        double degreesOff = Math.abs(targetHeading - currentHeading);
        
        if (degreesOff > 180) {
            goRight = !goRight;
            degreesOff = 360 - degreesOff;
        }

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

        double redThreshold = 40;
        double redDivisor = 2500;

        double outputValue = 0;

        leftColor = Robot.outsideLeft.red();
        rightColor = Robot.outsideRight.red();

        if (leftColor > redThreshold || rightColor > redThreshold) {
            outputValue = (rightColor - leftColor) / redDivisor;
        }

        return outputValue;
    }

    public boolean rightTurnAvailable() {
        return false;
    }
    public boolean leftTurnAvailable () {
        return false;
    }

    public void signalRightTurn() {
        telemetry.addLine("Right Turn Available");
    }
    public void signalLeftTurn() {
        telemetry.addLine("Left Turn Available");
    }
}