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
            
            if (pressingForward) {
                wheelPower = defaultWheelPower;
            } else {
                if (pressingReverse) {
                    wheelPower = -defaultWheelPower;
                } else {
                    wheelPower = 0;
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
    public double turn(double prevHeading, boolean turnRight) {

        double turnPower = .25;
        double turnReverse = 1;

        double targetHeading = prevHeading;
        if (turnRight) {
            targetHeading = targetHeading + 90;
        } else {
            targetHeading = targetHeading - 90;
        }
        targetHeading = (360 + targetHeading) % 360;
        
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentHeading = (360 + angles.firstAngle) % 360;
        boolean goRight = targetHeading > currentHeading;
        double degreesOff = Math.abs(targetHeading - currentHeading);

        if (degreesOff > 180) {
            goRight = !goRight;
            degreesOff = 360 - degreesOff;
        }
        
        while (degreesOff > .3) {

            if (goRight) {
                turnReverse = 1; // turn right
            } else {
                turnReverse = -1; // turn left
            }

            Robot.frontLeft.setPower(turnPower * turnReverse);
            Robot.backLeft.setPower(turnPower * turnReverse);
            Robot.frontRight.setPower(-turnPower * turnReverse);
            Robot.backRight.setPower(-turnPower * turnReverse);


            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = (360 + angles.firstAngle) % 360;
            goRight = targetHeading > currentHeading;
            degreesOff = Math.abs(targetHeading - currentHeading);

            if (degreesOff > 180) {
                goRight = !goRight;
                degreesOff = 360 - degreesOff;
            }
        }

        Robot.frontLeft.setPower(0);
        Robot.backLeft.setPower(0);
        Robot.frontRight.setPower(0);
        Robot.backRight.setPower(0);


        return targetHeading;
    }

    public void turnToHeading(double targetDegrees) {

        double turnPower = .25;
        double turnReverse = 1;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentHeading = (360 + angles.firstAngle) % 360;

        double degreesOff = targetDegrees - currentHeading;
        if(degreesOff > 180){
            degreesOff = 360 - degreesOff;
        }

        while (Math.abs(degreesOff) > .3) {

            if (currentHeading < targetDegrees) {
                turnReverse = 1;
            } else {
                turnReverse = -1;
            }

            Robot.frontLeft.setPower(turnPower * turnReverse);
            Robot.backLeft.setPower(turnPower * turnReverse);
            Robot.frontRight.setPower(-turnPower * turnReverse);
            Robot.backRight.setPower(-turnPower * turnReverse);
            
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = (360 + angles.firstAngle) % 360;
            degreesOff = targetDegrees - currentHeading;
            telemetry.addData("Target Degrees", targetDegrees);
            telemetry.update();
        }

        Robot.frontLeft.setPower(0);
        Robot.backLeft.setPower(0);
        Robot.frontRight.setPower(0);
        Robot.backRight.setPower(0);

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

        double redThreshold = 60;
        double redDivisor = 2500;

        double outputValue = 0;

        leftColor = Robot.midLeft.red();
        rightColor = Robot.midRight.red();

        if (leftColor > redThreshold || rightColor > redThreshold) {
            outputValue = (rightColor - leftColor) / redDivisor;
        }

        return outputValue;
    }

    public boolean rightTurnAvailable() {
        if(Robot.outsideRight.red() >= 90){
            return true;
        }
        else{
            return false;
        }
    }
    public boolean leftTurnAvailable () {
        if(Robot.outsideLeft.red() >= 105){
            return true;
        }
        else{
            return false;
        }
    }

    public void signalRightTurn() {
        telemetry.addLine("Right Turn Available");
    }
    public void signalLeftTurn() {
        telemetry.addLine("Left Turn Available");
    }
}