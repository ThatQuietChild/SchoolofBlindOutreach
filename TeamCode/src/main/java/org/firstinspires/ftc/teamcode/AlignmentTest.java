package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


@TeleOp

public class AlignmentTest extends LinearOpMode {



    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
        double midLeft = Robot.midLeft.red();
        double midRight = Robot.midRight.red();
            if(gamepad1.right_bumper){
                Robot.frontLeft.setPower(0.1);    // Strafe Right code
                Robot.backLeft.setPower(-0.1);
                Robot.frontRight.setPower(-0.1);
                Robot.backRight.setPower(0.1);

            }
            if (gamepad1.left_bumper){
                Robot.frontLeft.setPower(-0.1);    // Strafe Left code
                Robot.backLeft.setPower(0.1);
                Robot.frontRight.setPower(0.1);
                Robot.backRight.setPower(-0.1);
            }


            telemetry.addData("Mid Left", Robot.midLeft.red());
            telemetry.addData("Mid Right", Robot.midRight.red());
            telemetry.addData("Out Left", Robot.outsideLeft.red());
            telemetry.addData("Out Right", Robot.outsideRight.red());
            telemetry.update();
        }
    }
}