package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class mainOp extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor arm;
    private Servo grabber;

    private final double GRABBER_OPEN = 0;
    private final double GRABBER_CLOSED = 1;
    private final double PULSES_PER_REVOLUTION = 751.8; // gobilda 5202 223 rpm
    private final int ARM_LOWER_BOUND = - (int) (0.266 * PULSES_PER_REVOLUTION);
    private final int ARM_ZERO_POSITION = 0;
    private final int ARM_LOW_JUNCTION = (int) (3.857 * PULSES_PER_REVOLUTION);
    private final int ARM_MEDIUM_JUNCTION = (int) (6.385 * PULSES_PER_REVOLUTION);
    private final int ARM_HIGH_JUNCTION = (int) (8.779 * PULSES_PER_REVOLUTION);
    private final int ARM_UPPER_BOUND = (int) (8.779 * PULSES_PER_REVOLUTION);

    @Override
    public void runOpMode() throws InterruptedException {
        float x;
        float y;
        float clockwise;
        double fl;
        double fr;
        double bl;
        double br;
        int speed = 1;
        boolean slowmodeChanged = false;
        boolean shouldSlowmode = false;
        boolean isGrabbing = false;
        boolean changed = false;

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        arm = hardwareMap.get(DcMotor.class, "arm");
        grabber = hardwareMap.get(Servo.class, "grabber");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.REVERSE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        grabber.setPosition(GRABBER_OPEN);
        moveArm(ARM_ZERO_POSITION);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        AutoBase auto = new AutoBase(
                this,
                hardwareMap,
                "frontLeft",
                "frontRight",
                "backLeft",
                "backRight",
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

        waitForStart();
        if (opModeIsActive()) {
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            while (opModeIsActive()) {
                x = gamepad1.left_stick_x;
                y = -gamepad1.left_stick_y;
                clockwise = gamepad1.right_stick_x;

                if (gamepad1.dpad_up) {
                    y = (float) 1.0;
                } else if (gamepad1.dpad_down) {
                    y = (float) -1.0;
                }

                if (gamepad1.dpad_right) {
                    x = (float) 1.0;
                } else if (gamepad1.dpad_left) {
                    x = (float) -1.0;
                }

                if (gamepad1.back) {
                    clockwise = (float) -1.0;
                } else if (gamepad1.guide) {
                    clockwise = (float) 1.0;
                }

                fl = y + x + clockwise;
                fr = y - x - clockwise;
                bl = y - x + clockwise;
                br = y + x - clockwise;

                if (gamepad1.right_bumper) {
                    speed = 2;
                } else if (gamepad1.left_bumper) {
                    speed = 4;
                } else if (gamepad1.start) {
                    if (!slowmodeChanged) {
                        shouldSlowmode = !shouldSlowmode;
                        slowmodeChanged = true;
                    }
                } else {
                    if (slowmodeChanged) {
                        slowmodeChanged = false;
                    } else {
                        speed = 1;
                    }
                }
                if (shouldSlowmode) {
                    speed = 2;
                }

                if (gamepad1.left_stick_button) {
                    if (gamepad1.y) {
                        auto.turnLeft(45);
                    } else if (gamepad1.b) {
                        auto.turnLeft(120);
                    } else if (gamepad1.a) {
                        auto.turnLeft(180);
                    } else {
                        auto.turnLeft(90);
                    }
                } else if (gamepad1.right_stick_button) {
                    if (gamepad1.y) {
                        auto.turnRight(45);
                    } else if (gamepad1.b) {
                        auto.turnRight(120);
                    } else if (gamepad1.a) {
                        auto.turnRight(180);
                    } else {
                        auto.turnRight(90);
                    }
                }

                fl /= speed;
                fr /= speed;
                bl /= speed;
                br /= speed;

                frontLeft.setPower(fl);
                frontRight.setPower(fr);
                backLeft.setPower(bl);
                backRight.setPower(br);

                if (gamepad2.left_bumper) {
                    moveArm(ARM_ZERO_POSITION);
                } else if (gamepad2.a) {
                    moveArm(ARM_LOW_JUNCTION);
                } else if (gamepad2.b) {
                    moveArm(ARM_MEDIUM_JUNCTION);
                } else if (gamepad2.y) {
                    moveArm(ARM_HIGH_JUNCTION);
                } else if (gamepad2.right_bumper) {
                    moveArm(ARM_UPPER_BOUND);
                } else if (gamepad2.right_trigger > 0 || gamepad2.left_trigger > 0) {
                    if (arm.getCurrentPosition() < ARM_LOWER_BOUND) {
                        moveArm(ARM_LOWER_BOUND + 10);
                    } else if (arm.getCurrentPosition() > ARM_UPPER_BOUND) {
                        moveArm(ARM_UPPER_BOUND - 10);
                    } else {
                        moveArm((int) ((gamepad2.right_trigger - gamepad2.left_trigger) * 100) + arm.getCurrentPosition(), gamepad2.right_trigger - gamepad2.left_trigger);
                    }
                }
                telemetry.addData("Position", arm.getCurrentPosition());
                if (!arm.isBusy()) {
                    arm.setPower(0);
                    moveArm(arm.getTargetPosition());
                }

                if (gamepad2.x) {
                    if (!changed) {
                        isGrabbing = !isGrabbing;
                        changed = true;
                    }
                } else {
                    changed = false;
                }
                grabber.setPosition(isGrabbing ? GRABBER_CLOSED : GRABBER_OPEN);

                telemetry.update();
            }
        }
    }

    private void moveArm(int position) {
        moveArm(position, 1);
    }

    private void moveArm(int position, double motorPower) {
        arm.setTargetPosition(position);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(motorPower * ((position < arm.getCurrentPosition()) ? -1 : 1));
    }
}