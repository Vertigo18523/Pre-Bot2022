package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class AutoBase {
    static DcMotor frontLeft, frontRight, backLeft, backRight;

    private final Telemetry telemetry;
    private final LinearOpMode opMode;
    private OpenCvCamera camera = null;
    private SleeveDetection sleeveDetection = null;

    private static final double PULSES_PER_REVOLUTION = 384.5; // 435 rpm goBilda 5202
    private static final double WHEEL_DIAMETER_IN = 3.77953; // 96 mm
    private static final double PULSES_PER_IN = PULSES_PER_REVOLUTION / (WHEEL_DIAMETER_IN * Math.PI);
    private static double DRIVE_SPEED, TURN_SPEED, STRAFE_MULTIPLIER, DELAY_BETWEEN_METHODS, TURN_CONSTANT;
    private static boolean USE_PID;
    private final double kP, kI, kD;

    public AutoBase(
            LinearOpMode opMode,
            @NonNull HardwareMap hardwareMap,
            String leftFrontName,
            String rightFrontName,
            String leftBackName,
            String rightBackName,
            String cameraName,
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

        frontLeft = hardwareMap.get(DcMotor.class, leftFrontName);
        frontRight = hardwareMap.get(DcMotor.class, rightFrontName);
        backLeft = hardwareMap.get(DcMotor.class, leftBackName);
        backRight = hardwareMap.get(DcMotor.class, rightBackName);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        if (cameraName != null) {
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, cameraName), hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
            sleeveDetection = new SleeveDetection();
            camera.setPipeline(sleeveDetection);

            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
                }

                @Override
                public void onError(int errorCode) {
                }
            });

            while (!opMode.isStarted()) {
                telemetry.addData("Rotation", getSleeveRotation());
                telemetry.update();
            }
        }

        this.telemetry = telemetry;
        this.opMode = opMode;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public SleeveDetection.ParkingPosition getSleeveRotation() {
        return sleeveDetection.getPosition();
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
//        setMotors(motorPower);
        while (
                frontLeft.isBusy()
        ) {
            if (USE_PID) {
                proportional = totalTicks - frontLeft.getCurrentPosition();
                integral += proportional * timer.seconds();
                derivative = (proportional - prevError) / timer.seconds();
                pid = (kP * proportional) + (kI * integral) + (kD * derivative);
                setMotors(Math.min(pid, motorPower));
                prevError = proportional;
                timer.reset();
            } else {
//                opMode.idle();
                setMotors(totalTicks / 2.0 > frontLeft.getCurrentPosition() ? 1 : 0.5);
//                setMotors(((-4.0 * motorPower) / Math.pow(totalTicks, 2.0)) * Math.pow(totalTicks / 2.0 - frontLeft.getCurrentPosition(), 2.0) + motorPower);
                telemetry.addData("motorPower", frontLeft.getPower());
                telemetry.update();
            }
        }
        stopDriving();
        setRunWithoutEncoders();
        opMode.sleep((long) DELAY_BETWEEN_METHODS);
    }

    private interface goFunction {
        void run(int distanceTicks);
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
        turnLeft(90, TURN_SPEED);
    }

    public void turnLeft(int degrees) throws InterruptedException {
        turnLeft(degrees, TURN_SPEED);
    }

    public void turnLeft(int degrees, double motorPower) throws InterruptedException {
        drive(this::goTurnLeft, (int) (TURN_CONSTANT * degrees), motorPower);
    }

    public void turnRight() throws InterruptedException {
        turnRight(90, TURN_SPEED);
    }

    public void turnRight(int degrees) throws InterruptedException {
        turnRight(degrees, TURN_SPEED);
    }

    public void turnRight(int degrees, double motorPower) throws InterruptedException {
        drive(this::goTurnRight, (int) (TURN_CONSTANT * degrees), motorPower);
    }
}