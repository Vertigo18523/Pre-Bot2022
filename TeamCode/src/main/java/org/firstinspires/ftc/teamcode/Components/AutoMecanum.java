package org.firstinspires.ftc.teamcode.Components;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.Component;

public class AutoMecanum implements Component {
    private static final double PULSES_PER_REVOLUTION = 384.5; // 435 rpm goBilda 5202
    private static final double WHEEL_DIAMETER_IN = 3.77953; // 96 mm
    private static final double PULSES_PER_IN = PULSES_PER_REVOLUTION / (WHEEL_DIAMETER_IN * Math.PI);
    private static double DRIVE_SPEED, TURN_SPEED, STRAFE_MULTIPLIER, DELAY_BETWEEN_METHODS, TURN_CONSTANT;
    private static boolean USE_PID;
    private final LinearOpMode opMode;
    private final double kP, kI, kD;
    public Mecanum mecanum;

    public AutoMecanum(
            LinearOpMode opMode,
            String leftFrontName,
            String rightFrontName,
            String leftBackName,
            String rightBackName,
            @NonNull HardwareMap hardwareMap,
            Telemetry telemetry,
            double driveSpeed, // 1.0 power
            double turnSpeed, // 0.5 power
            double lengthInches, // front-back axle to axle
            double widthInches, // left-right wheel center to wheel center
            double strafeMultiplier, // 1.13 units
            double delay, // 100 ms
            boolean usePID, // https://www.ctrlaltftc.com/the-pid-controller/tuning-methods-of-a-pid-controller
            double kP,
            double kI,
            double kD
    ) {
        DRIVE_SPEED = driveSpeed;
        TURN_SPEED = turnSpeed;
        STRAFE_MULTIPLIER = strafeMultiplier;
        DELAY_BETWEEN_METHODS = delay;
        USE_PID = usePID;
        TURN_CONSTANT = (Math.PI * Math.sqrt((Math.pow(lengthInches / 2.0, 2.0) + Math.pow(widthInches / 2.0, 2.0)) / 2.0)) / 90.0;

        this.mecanum = new Mecanum(
                leftFrontName,
                leftBackName,
                rightFrontName,
                rightBackName,
                hardwareMap,
                telemetry
        );

        this.opMode = opMode;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    private void drive(@NonNull goFunction direction, double distanceIN, double motorPower) throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        double proportional, integral = 0, derivative, pid, prevError = 0, totalTicks = PULSES_PER_IN * distanceIN;
        resetEncoders();
        direction.run((int) totalTicks);
        if (USE_PID) {
            setRunWithoutEncoders();
        } else {
            setRunToPosition();
        }
        while (
                mecanum.frontLeft.isBusy()
        ) {
            if (USE_PID) {
                proportional = totalTicks - mecanum.frontLeft.getCurrentPosition();
                integral += proportional * timer.seconds();
                derivative = (proportional - prevError) / timer.seconds();
                pid = (kP * proportional) + (kI * integral) + (kD * derivative);
                setMotors(Math.min(pid, motorPower));
                prevError = proportional;
                timer.reset();
            } else {
                setMotors(motorPower);
                opMode.idle();
//                setMotors(totalTicks / 2.0 > mecanum.frontLeft.getCurrentPosition() ? 1 : 0.5);
//                setMotors(((-4.0 * motorPower) / Math.pow(totalTicks, 2.0)) * Math.pow(totalTicks / 2.0 - mecanum.frontLeft.getCurrentPosition(), 2.0) + motorPower);
                mecanum.telemetry.addData("motorPower", mecanum.frontLeft.getPower());
                mecanum.telemetry.update();
            }
        }
        stopDriving();
        setRunWithoutEncoders();
        opMode.sleep((long) DELAY_BETWEEN_METHODS);
    }

    @Override
    public void init() {
        mecanum.init();
    }

    @Override
    public void start() {
        mecanum.start();
    }

    @Override
    public void update() {
        mecanum.update();
    }

    private void setRunToPosition() {
        mecanum.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mecanum.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mecanum.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mecanum.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void resetEncoders() {
        mecanum.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mecanum.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mecanum.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mecanum.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setRunWithoutEncoders() {
        mecanum.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mecanum.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mecanum.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mecanum.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void goForward(Object distance) {
        mecanum.frontLeft.setTargetPosition((int) distance);
        mecanum.frontRight.setTargetPosition((int) distance);
        mecanum.backLeft.setTargetPosition((int) distance);
        mecanum.backRight.setTargetPosition((int) distance);
    }

    private void goBackward(Object distance) {
        mecanum.frontLeft.setTargetPosition(-(int) distance);
        mecanum.frontRight.setTargetPosition(-(int) distance);
        mecanum.backLeft.setTargetPosition(-(int) distance);
        mecanum.backRight.setTargetPosition(-(int) distance);
    }

    private void goLeft(Object distance) {
        mecanum.frontLeft.setTargetPosition(-(int) distance);
        mecanum.frontRight.setTargetPosition((int) distance);
        mecanum.backLeft.setTargetPosition((int) distance);
        mecanum.backRight.setTargetPosition(-(int) distance);
    }

    private void goRight(Object distance) {
        mecanum.frontLeft.setTargetPosition((int) distance);
        mecanum.frontRight.setTargetPosition(-(int) distance);
        mecanum.backLeft.setTargetPosition(-(int) distance);
        mecanum.backRight.setTargetPosition((int) distance);
    }

    private void goNW(Object distance) {
        mecanum.frontLeft.setTargetPosition(0);
        mecanum.frontRight.setTargetPosition((int) distance);
        mecanum.backLeft.setTargetPosition((int) distance);
        mecanum.backRight.setTargetPosition(0);
    }

    private void goNE(Object distance) {
        mecanum.frontLeft.setTargetPosition((int) distance);
        mecanum.frontRight.setTargetPosition(0);
        mecanum.backLeft.setTargetPosition(0);
        mecanum.backRight.setTargetPosition((int) distance);
    }

    private void goSW(Object distance) {
        mecanum.frontLeft.setTargetPosition(-(int) distance);
        mecanum.frontRight.setTargetPosition(0);
        mecanum.backLeft.setTargetPosition(0);
        mecanum.backRight.setTargetPosition(-(int) distance);
    }

    private void goSE(Object distance) {
        mecanum.frontLeft.setTargetPosition(0);
        mecanum.frontRight.setTargetPosition(-(int) distance);
        mecanum.backLeft.setTargetPosition(-(int) distance);
        mecanum.backRight.setTargetPosition(0);
    }

    private void goTurnLeft(Object distance) {
        mecanum.frontLeft.setTargetPosition(-(int) distance);
        mecanum.frontRight.setTargetPosition((int) distance);
        mecanum.backLeft.setTargetPosition(-(int) distance);
        mecanum.backRight.setTargetPosition((int) distance);
    }

    private void goTurnRight(Object distance) {
        mecanum.frontLeft.setTargetPosition((int) distance);
        mecanum.frontRight.setTargetPosition(-(int) distance);
        mecanum.backLeft.setTargetPosition((int) distance);
        mecanum.backRight.setTargetPosition(-(int) distance);
    }

    private void stopDriving() {
        mecanum.frontLeft.setPower(0);
        mecanum.frontRight.setPower(0);
        mecanum.backLeft.setPower(0);
        mecanum.backRight.setPower(0);
    }

    private void setMotors(double power) {
        mecanum.frontLeft.setPower(power);
        mecanum.frontRight.setPower(power);
        mecanum.backLeft.setPower(power);
        mecanum.backRight.setPower(power);
    }

    public void driveForward(double distanceIN) throws InterruptedException {
        driveForward(distanceIN, DRIVE_SPEED);
    }

    public void driveForward(double distanceIN, double motorPower) throws InterruptedException {
        drive(this::goForward, distanceIN, motorPower);
    }

    public void driveBackward(double distanceIN) throws InterruptedException {
        driveBackward(distanceIN, DRIVE_SPEED);
    }

    public void driveBackward(double distanceIN, double motorPower) throws InterruptedException {
        drive(this::goBackward, distanceIN, motorPower);
    }

    public void strafeLeft(double distanceIN) throws InterruptedException {
        strafeLeft(distanceIN, DRIVE_SPEED);
    }

    public void strafeLeft(double distanceIN, double motorPower) throws InterruptedException {
        drive(this::goLeft, distanceIN * STRAFE_MULTIPLIER, motorPower);
    }

    public void strafeRight(double distanceIN) throws InterruptedException {
        strafeRight(distanceIN, DRIVE_SPEED);
    }

    public void strafeRight(double distanceIN, double motorPower) throws InterruptedException {
        drive(this::goRight, distanceIN * STRAFE_MULTIPLIER, motorPower);
    }

    public void strafeNW(double distanceIN) throws InterruptedException {
        strafeNW(distanceIN, DRIVE_SPEED);
    }

    public void strafeNW(double distanceIN, double motorPower) throws InterruptedException {
        drive(this::goNW, distanceIN, motorPower);
    }

    public void strafeNE(double distanceIN) throws InterruptedException {
        strafeNE(distanceIN, DRIVE_SPEED);
    }

    public void strafeNE(double distanceIN, double motorPower) throws InterruptedException {
        drive(this::goNE, distanceIN, motorPower);
    }

    public void strafeSW(double distanceIN) throws InterruptedException {
        strafeSW(distanceIN, DRIVE_SPEED);
    }

    public void strafeSW(double distanceIN, double motorPower) throws InterruptedException {
        drive(this::goSW, distanceIN, motorPower);
    }

    public void strafeSE(double distanceIN) throws InterruptedException {
        strafeSE(distanceIN, DRIVE_SPEED);
    }

    public void strafeSE(double distanceIN, double motorPower) throws InterruptedException {
        drive(this::goSE, distanceIN, motorPower);
    }

    public void turnLeft() throws InterruptedException {
        turnLeft(90);
    }

    public void turnLeft(int degrees) throws InterruptedException {
        turnLeft(degrees, TURN_SPEED);
    }

    public void turnLeft(int degrees, double motorPower) throws InterruptedException {
        drive(this::goTurnLeft, (int) (TURN_CONSTANT * degrees), motorPower);
    }

    public void turnRight() throws InterruptedException {
        turnRight(90);
    }

    public void turnRight(int degrees) throws InterruptedException {
        turnRight(degrees, TURN_SPEED);
    }

    public void turnRight(int degrees, double motorPower) throws InterruptedException {
        drive(this::goTurnRight, (int) (TURN_CONSTANT * degrees), motorPower);
    }

    private interface goFunction {
        void run(int distanceTicks);
    }
}