package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.awt.font.NumericShaper;

@TeleOp(name = "color sensor test")

public class ColorSensorTest extends LinearOpMode {
    BNO055IMU imu;
    Orientation angles;

    @Override
    public void runOpMode() throws InterruptedException {
        org.firstinspires.ftc.teamcode.Robot robot = new org.firstinspires.ftc.teamcode.Robot(hardwareMap);

        initIMU();

        waitForStart();
        double slowfactor = 0.5;
        Gamepad.RumbleEffect rumbleLeft;
       rumbleLeft = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 0.0, 500)  //  Rumble left motor 100% for 500 mSec

                .build();

        Gamepad.RumbleEffect rumbleRight;
        rumbleRight = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec

                .build();

        while (opModeIsActive()) {

            double leftStickY = gamepad1.left_stick_y * -1;
            double leftStickX = gamepad1.left_stick_x;
            double rightStickX = gamepad1.right_stick_x;

            double wheelPower;
            double stickAngleRadians;
            double rightX;
            double lfPower;
            double rfPower;
            double lrPower;
            double rrPower;


            wheelPower = Math.hypot(leftStickX, leftStickY);
            if (wheelPower > .02) {
                wheelPower = (.8 * wheelPower + .2) * slowfactor;
            }


            stickAngleRadians = Math.atan2(leftStickY, leftStickX);

            stickAngleRadians = stickAngleRadians - Math.PI / 4; //adjust by 45 degrees

            double sinAngleRadians = Math.sin(stickAngleRadians);
            double cosAngleRadians = Math.cos(stickAngleRadians);
            double factor = 1 / Math.max(Math.abs(sinAngleRadians), Math.abs(cosAngleRadians));

            rightX = rightStickX * slowfactor * .8;

            lfPower = wheelPower * cosAngleRadians * factor + rightX;
            rfPower = wheelPower * sinAngleRadians * factor - rightX;
            lrPower = wheelPower * sinAngleRadians * factor + rightX;
            rrPower = wheelPower * cosAngleRadians * factor - rightX;

            robot.backLeft.setPower(lrPower);
            robot.backRight.setPower(rrPower);
            robot.frontLeft.setPower(lfPower);
            robot.frontRight.setPower(rfPower);

            /**if(Robot.midLeft.red() != Range.clip(167, 157, 177)) {

                align(167, 134);
            }
            else if(Robot.midRight.red() != Range.clip(134, 124, 144)){
                align(167, 134);
            }
            else{
                break;
            }**/




            if (Robot.outsideLeft.red()>=80){

                gamepad1.runRumbleEffect(rumbleLeft);

            }
            if (Robot.outsideRight.red()>=80){

                gamepad1.runRumbleEffect(rumbleRight);

            }

            if (Robot.outsideLeft.red()>= 30 && gamepad1.left_bumper) {
                Turn(90);
            }
            if (Robot.outsideRight.red() >= 30 && gamepad1.right_bumper) {
                Turn(-90);
            }

            //telemetry.addData("RL Servo: ", tapeMeasureRLServoPosition);
            //telemetry.addData("UD Servo: ", tapeMeasureUDServoPosition);
            telemetry.addData("LeftStickX", gamepad1.left_stick_x);
            telemetry.addData("LeftStickY", gamepad1.left_stick_y);
            telemetry.addData("RightStickX", gamepad1.right_stick_x);
            telemetry.addData("RightStickY", gamepad1.right_stick_y);
            telemetry.addData("Mid Left", Robot.midLeft.red());
            telemetry.addData("Mid Right", Robot.midRight.red());
            telemetry.addData("Out Left", Robot.outsideLeft.red());
            telemetry.addData("Out Right", Robot.outsideRight.red());
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Current angle", angles.firstAngle);
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

    public void Turn(double targetDegrees) {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentAngle = angles.firstAngle;
        while (true) {

            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentAngle = angles.firstAngle;

            if (currentAngle < targetDegrees) {
                Robot.frontRight.setPower(0.25);
                Robot.backRight.setPower(0.25);
                Robot.frontLeft.setPower(-0.25);
                Robot.backLeft.setPower(-0.25);
            }

            if (currentAngle > targetDegrees) {
                Robot.frontRight.setPower(-0.25);
                Robot.backRight.setPower(-0.25);
                Robot.frontLeft.setPower(0.25);
                Robot.backLeft.setPower(0.25);
            }
            if (currentAngle + 3 > targetDegrees && currentAngle - 3 < targetDegrees) {
                break;
            }
        }

        Robot.frontRight.setPower(0);
        Robot.backRight.setPower(0);
        Robot.frontLeft.setPower(0);
        Robot.backLeft.setPower(0);

        initIMU();
    }

    /**public void align(double targetColorLeft, double targetColorRight){
        double midLeft = Robot.midLeft.red();
        double midRight = Robot.midRight.red();
        while(true) {
            if (midLeft < targetColorLeft) {
                Robot.frontRight.setPower(0.2);
                Robot.backRight.setPower(0.2);
                Robot.frontLeft.setPower(0.2);
                Robot.backLeft.setPower(0.2);
            } else if (midRight < targetColorRight) {
                Robot.frontLeft.setPower(0.2);
                Robot.backLeft.setPower(-0.2);
                Robot.frontRight.setPower(-0.2);
                Robot.backRight.setPower(0.2);
            } else if (midLeft > targetColorLeft) {
                Robot.frontLeft.setPower(0.2);
                Robot.backLeft.setPower(-0.2);
                Robot.frontRight.setPower(-0.2);
                Robot.backRight.setPower(0.2);
            } else if (midRight > targetColorRight) {
                Robot.frontRight.setPower(0.2);
                Robot.backRight.setPower(0.2);
                Robot.frontLeft.setPower(0.2);
                Robot.backLeft.setPower(0.2);
            } else {
                Robot.frontRight.setPower(0);
                Robot.backRight.setPower(0);
                Robot.frontLeft.setPower(0);
                Robot.backLeft.setPower(0);
            }

            if (midLeft + 10 > targetColorLeft && midLeft - 10< targetColorLeft) {
                break;
            }
            if (midRight + 10 > targetColorRight && midRight - 10 < targetColorRight) {
                break;
            }

        }**/



    }


