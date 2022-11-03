package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Base.BaseOpMode;
import org.firstinspires.ftc.teamcode.Base.Robot;
import org.firstinspires.ftc.teamcode.Bots.PreBot;

@TeleOp
public class MainOp extends BaseOpMode {
    public PreBot robot;

    @Override
    protected Robot setRobot() {
        this.robot = new PreBot();
        return this.robot;
    }

    @Override
    protected boolean setTeleOp() {
        return true;
    }

    @Override
    public void onUpdate() throws InterruptedException {
        robot.mecanum.mecanum.setInitialDirections(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if (gamepad1.dpad_up) {
            robot.mecanum.mecanum.driveForward();
        } else if (gamepad1.dpad_down) {
            robot.mecanum.mecanum.driveBackward();
        }

        if (gamepad1.dpad_right) {
            robot.mecanum.mecanum.strafeRight();
        } else if (gamepad1.dpad_left) {
            robot.mecanum.mecanum.strafeLeft();
        }

        if (gamepad1.back) {
            robot.mecanum.mecanum.turnLeft();
        } else if (gamepad1.guide) {
            robot.mecanum.mecanum.turnRight();
        }

        robot.mecanum.mecanum.setNewDirections();

        if (gamepad1.right_bumper) {
            robot.mecanum.mecanum.setHalfSpeed();
        } else if (gamepad1.left_bumper) {
            robot.mecanum.mecanum.setQuarterSpeed();
        } else if (gamepad1.start) {
            robot.mecanum.mecanum.buttonPressed();
        } else {
            robot.mecanum.mecanum.buttonReleased();
        }

        if (gamepad1.left_stick_button) {
            robot.mecanum.turnLeft();
        } else if (gamepad1.right_stick_button) {
            robot.mecanum.turnRight();
        }

        if (gamepad2.left_bumper) {
            robot.arm.move(robot.arm.ZERO_POSITION);
        } else if (gamepad2.a) {
            robot.arm.move(robot.arm.LOW_JUNCTION);
        } else if (gamepad2.b) {
            robot.arm.move(robot.arm.MEDIUM_JUNCTION);
        } else if (gamepad2.y) {
            robot.arm.move(robot.arm.HIGH_JUNCTION);
        } else if (gamepad2.right_bumper) {
            robot.arm.move(robot.arm.UPPER_BOUND);
        } else if (gamepad2.right_trigger > 0 || gamepad2.left_trigger > 0) {
            if (robot.arm.getCurrentPosition() < robot.arm.LOWER_BOUND) {
                robot.arm.move(robot.arm.LOWER_BOUND + 10);
            } else if (robot.arm.getCurrentPosition() > robot.arm.UPPER_BOUND) {
                robot.arm.move(robot.arm.UPPER_BOUND - 10);
            } else {
                robot.arm.move((int) ((gamepad2.right_trigger - gamepad2.left_trigger) * 100) + robot.arm.getCurrentPosition(), gamepad2.right_trigger - gamepad2.left_trigger);
            }
        }

        if (gamepad2.x) {
            robot.grabber.buttonPressed();
        } else {
            robot.grabber.buttonReleased();
        }

        telemetry.update();
    }
}

