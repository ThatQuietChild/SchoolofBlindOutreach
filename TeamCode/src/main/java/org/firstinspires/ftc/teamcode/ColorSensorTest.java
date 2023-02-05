package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "color sensor test")

public class ColorSensorTest extends LinearOpMode {
    BNO055IMU imu;
    Orientation angles;

    @Override
    public void runOpMode() throws InterruptedException {
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

        while (opModeIsActive()) {

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
}
