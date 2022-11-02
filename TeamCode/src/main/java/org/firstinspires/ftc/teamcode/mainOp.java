package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.BaseOpMode;
import org.firstinspires.ftc.teamcode.Base.Robot;
import org.firstinspires.ftc.teamcode.Components.Arm;
import org.firstinspires.ftc.teamcode.Components.Grabber;
import org.firstinspires.ftc.teamcode.Components.AutoMecanum;

@TeleOp
public class mainOp extends Robot {
    @Override
    protected void initBot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode) {

        final AutoMecanum mecanum = new AutoMecanum(
                opMode,
                "frontLeft",
                "frontRight",
                "backLeft",
                "backRight",
                hardwareMap,
                telemetry,
                1,
                1,
                10.5,
                12.5,
                1.13,
                100,
                false,
                0,
                0,
                0
        );
        final Arm arm = new Arm("arm", hardwareMap, telemetry);
        final Grabber grabber = new Grabber("grabber", hardwareMap, telemetry);

        addComponents(mecanum, arm, grabber);
    }

        while (opModeIsActive()) {
            // drivetrain
            if (gamepad1.dpad_up) {
                mecanum.mecanum.driveForward();
            } else if (gamepad1.dpad_down) {
                mecanum.mecanum.driveBackward();
            }

            if (gamepad1.dpad_right) {
                mecanum.mecanum.strafeRight();
            } else if (gamepad1.dpad_left) {
                mecanum.mecanum.strafeLeft();
            }

            if (gamepad1.back) {
                mecanum.mecanum.turnLeft();
            } else if (gamepad1.guide) {
                mecanum.mecanum.turnRight();
            }

            mecanum.setInitialDirections(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            if (gamepad1.right_bumper) {
                mecanum.setHalfSpeed();
            } else if (gamepad1.left_bumper) {
                mecanum.setQuarterSpeed();
            } else if (gamepad1.start) {
                mecanum.buttonPressed();
            } else {
                mecanum.buttonReleased();
            }

            mecanum.update();

            if (gamepad1.left_stick_button) {
                mecanum.turnLeft();
            } else if (gamepad1.right_stick_button) {
                mecanum.turnRight();
            }

            // arm
            if (gamepad2.left_bumper) {
                arm.move(arm.ZERO_POSITION);
            } else if (gamepad2.a) {
                arm.move(arm.LOW_JUNCTION);
            } else if (gamepad2.b) {
                arm.move(arm.MEDIUM_JUNCTION);
            } else if (gamepad2.y) {
                arm.move(arm.HIGH_JUNCTION);
            } else if (gamepad2.right_bumper) {
                arm.move(arm.UPPER_BOUND);
            } else if (gamepad2.right_trigger > 0 || gamepad2.left_trigger > 0) {
                if (arm.getCurrentPosition() < arm.LOWER_BOUND) {
                    arm.move(arm.LOWER_BOUND + 10);
                } else if (arm.getCurrentPosition() > arm.UPPER_BOUND) {
                    arm.move(arm.UPPER_BOUND - 10);
                } else {
                    arm.move((int) ((gamepad2.right_trigger - gamepad2.left_trigger) * 100) + arm.getCurrentPosition(), gamepad2.right_trigger - gamepad2.left_trigger);
                }
            }
//                new Action(arm::update, true);
            arm.update();

            // grabber
            if (gamepad2.x) {
                grabber.buttonPressed();
            } else {
                grabber.buttonReleased();
            }
//                new Action(grabber::update, true);
            grabber.update();

            telemetry.update();
        }
    }
}