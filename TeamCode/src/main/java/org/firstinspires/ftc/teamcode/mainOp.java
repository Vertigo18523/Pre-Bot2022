/*
Control Scheme:
  Gamepad 1 - robot locomotion:
    left stick - xy position of robot
    right stick - rotation of robot
    right bumper 1/2 speed slowmode
    dpad - 1.0 power in any given direction
*/

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
    private final int ARM_LOWER_BOUND = 0; // tbd
    private final int ARM_LOW_JUNCTION = 1; // tbd
    private final int ARM_MEDIUM_JUNCTION = 2; // tbd
    private final int ARM_HIGH_JUNCTION = 3; // tbd
    private final int ARM_UPPER_BOUND = 4; // tbd

    @Override
    public void runOpMode() {
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
        boolean isGrabbing = true;
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
        grabber.setDirection(Servo.Direction.REVERSE);

        grabber.setPosition(GRABBER_OPEN);
        moveArm(ARM_LOWER_BOUND);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
                14,
                13.5,
                1.13,
                100
        );

        waitForStart();
        if (opModeIsActive()) {
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            telemetry.addData("Control Scheme",
                    "\nleft stick - xy position of robot\nright stick - rotation of robot\nright bumper - 1/2 speed slowmode\nleft bumper - 1/4 speed slowmode\ndpad - 1.0 power in any given direction\nclick stick - rotate that direction\n\tnone - 90\n\ty - 45\n\tb - 120\n\ta - 180\nback - turn left\nguide - turn right");

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
                    moveArm(ARM_LOWER_BOUND);
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
                        moveArm(ARM_LOWER_BOUND);
                    } else if (arm.getCurrentPosition() > ARM_UPPER_BOUND) {
                        moveArm(ARM_UPPER_BOUND);
                    } else {
                        moveArm((int) (arm.getCurrentPosition() + gamepad2.right_trigger - gamepad2.left_trigger),gamepad2.right_trigger - gamepad2.left_trigger);
                    }
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
        while (arm.isBusy()) {
            idle();
        }
        arm.setPower(0);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}