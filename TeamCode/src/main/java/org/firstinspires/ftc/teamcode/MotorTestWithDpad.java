package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Trigger/Bumper Motor/Encoder test")
//@Disabled
public class MotorTestWithDpad extends LinearOpMode {

    double triggerSensitivity = 0.01;

    @Override
    public void runOpMode() {

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_trigger > triggerSensitivity) {
                Robot.frontLeft.setPower(.3);
            } else {
                Robot.frontLeft.setPower(0);
            }
            if (gamepad1.right_trigger > triggerSensitivity) {
                Robot.frontRight.setPower(.3);
            } else {
                Robot.frontRight.setPower(0);
            }
            if (gamepad1.left_bumper) {
                Robot.backLeft.setPower(.3);
            } else {
                Robot.backLeft.setPower(0);
            }
            if (gamepad1.right_bumper) {
                Robot.backRight.setPower(.3);
            } else {
                Robot.backRight.setPower(0);
            }

            telemetry.addData("FrontLeft:", Robot.frontLeft.getCurrentPosition());
            telemetry.addData("FrontRight:", Robot.frontRight.getCurrentPosition());
            telemetry.addData("BackLeft:", Robot.backLeft.getCurrentPosition());
            telemetry.addData("BackRight:", Robot.backRight.getCurrentPosition());

            telemetry.update();
        }
    }
}

