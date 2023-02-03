package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;


@TeleOp

public class SchoolofBlind extends LinearOpMode {



    @Override
    public void runOpMode() {
        org.firstinspires.ftc.teamcode.Robot robot = new org.firstinspires.ftc.teamcode.Robot(hardwareMap);
        waitForStart();
        double slowfactor = 0.5;

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

            //telemetry.addData("RL Servo: ", tapeMeasureRLServoPosition);
            //telemetry.addData("UD Servo: ", tapeMeasureUDServoPosition);
            telemetry.addData("LeftStickX", gamepad1.left_stick_x);
            telemetry.addData("LeftStickY", gamepad1.left_stick_y);
            telemetry.addData("RightStickX", gamepad1.right_stick_x);
            telemetry.addData("RightStickY", gamepad1.right_stick_y);
            telemetry.update();
        }
    }
}