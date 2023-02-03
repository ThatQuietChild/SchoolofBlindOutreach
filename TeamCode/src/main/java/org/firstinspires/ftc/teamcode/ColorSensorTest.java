package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "color sensor test")

public class ColorSensorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        org.firstinspires.ftc.teamcode.Robot robot = new org.firstinspires.ftc.teamcode.Robot(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {

            if (Robot.outsideLeft.red()>=80){
                gamepad1.rumble(10);
                sleep(1000)
            }
            if (Robot.outsideLeft.red()>=80 && gamepad1.left_bumper){
                Robot.frontRight.setPower(0.5);
                Robot.frontLeft.setPower(-0.5);
                Robot.backLeft.setPower(-0.5);
                Robot.backRight.setPower(0.5);
            }
            telemetry.addData("Mid Left", Robot.midLeft.red());
            telemetry.addData("Mid Right", Robot.midRight.red());

            telemetry.addData("Out Left", Robot.outsideLeft.red());
            telemetry.addData("Out Right", Robot.outsideRight.red());

            telemetry.update();

        }
    }
}
