package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoBase {
    static DcMotor frontLeft, frontRight, backLeft, backRight;

    Telemetry telemetry;
    LinearOpMode opMode;

    static final double PULSES_PER_REVOLUTION = 384.5; // 435 rpm goBilda 5202
    static final double WHEEL_DIAMETER_IN = 3.77953; // 96 mm
    static final double PULSES_PER_IN = PULSES_PER_REVOLUTION / (WHEEL_DIAMETER_IN * Math.PI);
    static double DRIVE_SPEED, TURN_SPEED, STRAFE_MULTIPLIER, DELAY_BETWEEN_METHODS, TURN_CONSTANT;

    public AutoBase(
            LinearOpMode opMode,
            HardwareMap hardwareMap,
            String left_front_name,
            String right_front_name,
            String left_back_name,
            String right_back_name,
            Telemetry telemetry,
            double driveSpeed, // 1.0
            double turnSpeed, // 0.5
            double lengthInches, // front-back axle to axle
            double widthInches, // left-right wheel center to wheel center
            double strafeMultiplier, // 1.13
            double delay // 100
    ) {
        DRIVE_SPEED = driveSpeed;
        TURN_SPEED = turnSpeed;
        STRAFE_MULTIPLIER = strafeMultiplier;
        DELAY_BETWEEN_METHODS = delay;
        TURN_CONSTANT = (Math.PI * Math.sqrt((Math.pow(lengthInches / 2.0, 2.0) + Math.pow(widthInches / 2.0, 2.0)) / 2.0)) / 90.0;

        frontLeft = hardwareMap.get(DcMotor.class, left_front_name);
        frontRight = hardwareMap.get(DcMotor.class, right_front_name);
        backLeft = hardwareMap.get(DcMotor.class, left_back_name);
        backRight = hardwareMap.get(DcMotor.class, right_back_name);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        this.telemetry = telemetry;
        this.opMode = opMode;
    }
    private void sleep(double milliseconds) {
        opMode.sleep((long) milliseconds);
    }

    private void idle() {
        opMode.idle();
    }

    private void print(String key, String value) {
        this.telemetry.addData(key, value);
        this.telemetry.update();
    }

    private static void setRunToPosition() {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private static void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private static void setRunWithoutEncoders() {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void goForward(Object distance) {
        frontLeft.setTargetPosition((int) distance);
        frontRight.setTargetPosition((int) distance);
        backLeft.setTargetPosition((int) distance);
        backRight.setTargetPosition((int) distance);
    }

    private void goBackward(Object distance) {
        frontLeft.setTargetPosition(-(int) distance);
        frontRight.setTargetPosition(-(int) distance);
        backLeft.setTargetPosition(-(int) distance);
        backRight.setTargetPosition(-(int) distance);
    }

    private void goLeft(Object distance) {
        frontLeft.setTargetPosition(-(int) distance);
        frontRight.setTargetPosition((int) distance);
        backLeft.setTargetPosition((int) distance);
        backRight.setTargetPosition(-(int) distance);
    }

    private void goRight(Object distance) {
        frontLeft.setTargetPosition((int) distance);
        frontRight.setTargetPosition(-(int) distance);
        backLeft.setTargetPosition(-(int) distance);
        backRight.setTargetPosition((int) distance);
    }

    private void goNW(Object distance) {
        frontLeft.setTargetPosition(0);
        frontRight.setTargetPosition((int) distance);
        backLeft.setTargetPosition((int) distance);
        backRight.setTargetPosition(0);
    }

    private void goNE(Object distance) {
        frontLeft.setTargetPosition((int) distance);
        frontRight.setTargetPosition(0);
        backLeft.setTargetPosition(0);
        backRight.setTargetPosition((int) distance);
    }

    private void goSW(Object distance) {
        frontLeft.setTargetPosition(-(int) distance);
        frontRight.setTargetPosition(0);
        backLeft.setTargetPosition(0);
        backRight.setTargetPosition(-(int) distance);
    }

    private void goSE(Object distance) {
        frontLeft.setTargetPosition(0);
        frontRight.setTargetPosition(-(int) distance);
        backLeft.setTargetPosition(-(int) distance);
        backRight.setTargetPosition(0);
    }

    private void goTurnLeft(Object distance) {
        frontLeft.setTargetPosition(-(int) distance);
        frontRight.setTargetPosition((int) distance);
        backLeft.setTargetPosition(-(int) distance);
        backRight.setTargetPosition((int) distance);
    }

    private void goTurnRight(Object distance) {
        frontLeft.setTargetPosition((int) distance);
        frontRight.setTargetPosition(-(int) distance);
        backLeft.setTargetPosition((int) distance);
        backRight.setTargetPosition(-(int) distance);
    }

    private static void stopDriving() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private static void setMotors(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    private void drive(goFunction direction, double distanceIN, double motorPower) {
        resetEncoders();
        direction.run((int) (PULSES_PER_IN*distanceIN));
        setRunToPosition();
        setMotors(motorPower);
        while (
                frontLeft.isBusy() &&
                        frontRight.isBusy() &&
                        backLeft.isBusy() &&
                        backRight.isBusy()
        ) {
            idle();
        }
        stopDriving();
        setRunWithoutEncoders();
        sleep(DELAY_BETWEEN_METHODS);
    }

    private interface goFunction {
        void run(int distanceIN);
    }

    public void driveForward(double distanceIN) {
        driveForward(distanceIN, DRIVE_SPEED);
    }

    public void driveForward(double distanceIN, double motorPower) {
        drive(this::goForward, distanceIN, motorPower);
    }

    public void driveBackward(double distanceIN) {
        driveBackward(distanceIN, DRIVE_SPEED);
    }

    public void driveBackward(double distanceIN, double motorPower) {
        drive(this::goBackward, distanceIN, motorPower);
    }

    public void strafeLeft(double distanceIN) {
        strafeLeft(distanceIN, DRIVE_SPEED);
    }

    public void strafeLeft(double distanceIN, double motorPower) {
        drive(this::goLeft, distanceIN * STRAFE_MULTIPLIER, motorPower);
    }

    public void strafeRight(double distanceIN) {
        strafeRight(distanceIN, DRIVE_SPEED);
    }

    public void strafeRight(double distanceIN, double motorPower) {
        drive(this::goRight, distanceIN * STRAFE_MULTIPLIER, motorPower);
    }

    public void strafeNW(double distanceIN) {
        strafeNW(distanceIN, DRIVE_SPEED);
    }

    public void strafeNW(double distanceIN, double motorPower) {
        drive(this::goNW, distanceIN, motorPower);
    }

    public void strafeNE(double distanceIN) {
        strafeNE(distanceIN, DRIVE_SPEED);
    }

    public void strafeNE(double distanceIN, double motorPower) {
        drive(this::goNE, distanceIN, motorPower);
    }

    public void strafeSW(double distanceIN) {
        strafeSW(distanceIN, DRIVE_SPEED);
    }

    public void strafeSW(double distanceIN, double motorPower) {
        drive(this::goSW, distanceIN, motorPower);
    }

    public void strafeSE(double distanceIN) {
        strafeSE(distanceIN, DRIVE_SPEED);
    }

    public void strafeSE(double distanceIN, double motorPower) {
        drive(this::goSE, distanceIN, motorPower);
    }

    public void turnLeft() {
        turnLeft(90, TURN_SPEED);
    }

    public void turnLeft(int degrees) {
        turnLeft(degrees, TURN_SPEED);
    }

    public void turnLeft(int degrees, double motorPower) {
        drive(this::goTurnLeft, (int) (TURN_CONSTANT * degrees), motorPower);
    }

    public void turnRight() {
        turnRight(90, TURN_SPEED);
    }

    public void turnRight(int degrees) {
        turnRight(degrees, TURN_SPEED);
    }

    public void turnRight(int degrees, double motorPower) {
        drive(this::goTurnRight, (int) (TURN_CONSTANT * degrees), motorPower);
    }
}

/*
    // During init:
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
        18,
        18,
        1.13,
        100
    );

    // During run loop:
    auto.driveForward(12);
    auto.driveBackward(12);
    auto.strafeLeft(12);
    auto.strafeRight(12);
    auto.strafeNW(12);
    auto.strafeNE(12);
    auto.strafeSW(12);
    auto.strafeSE(12);
    auto.turnLeft(90);
    auto.turnRight(90);
*/