package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.Component;

public class Arm implements Component {
    private final DcMotor arm;

    private final Telemetry telemetry;

    public double PULSES_PER_REVOLUTION;
    public int LOWER_BOUND;
    public int ZERO_POSITION;
    public int LOW_JUNCTION;
    public int MEDIUM_JUNCTION;
    public int HIGH_JUNCTION;
    public int UPPER_BOUND;

    public double MotorPower;
    public double TotalTicks;
    public double StartingPosition;

    public Arm(String deviceName, HardwareMap hardwareMap, Telemetry telemetry) {
        arm = hardwareMap.get(DcMotor.class, deviceName);
        this.PULSES_PER_REVOLUTION = 384.5; // gobilda 5202 435 rpm
        this.LOWER_BOUND = -(int) (0.266 * PULSES_PER_REVOLUTION);
        this.ZERO_POSITION = 0;
        this.LOW_JUNCTION = (int) (3.857 * PULSES_PER_REVOLUTION);
        this.MEDIUM_JUNCTION = (int) (6.385 * PULSES_PER_REVOLUTION);
        this.HIGH_JUNCTION = (int) (9.100 * PULSES_PER_REVOLUTION);
        this.UPPER_BOUND = (int) (9.100 * PULSES_PER_REVOLUTION);
        this.telemetry = telemetry;
    }

    @Override
    public void init() {
        arm.setDirection(DcMotor.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        move(ZERO_POSITION);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {
    }

    @Override
    public void update() {
        telemetry.addData("Position", getCurrentPosition());
        if (isBusy()) {
            setPower(MotorPower);
//            if (Math.max(TotalTicks, StartingPosition) == TotalTicks) {
//                setPower(TotalTicks / 2.0 > getCurrentPosition() ? 1 : 0.1);
//            } else {
//                setPower(TotalTicks / 2.0 < getCurrentPosition() ? 1 : 0.1);
//            }
//            setPower(((-4.0 * MotorPower) / Math.pow(TotalTicks, 2.0)) * Math.pow(TotalTicks / 2.0 - getCurrentPosition(), 2.0) + MotorPower);
        } else {
            arm.setPower(0);
            move(getTargetPosition());
        }
    }

    public void move(int position) {
        move(position, 1);
    }

    public void move(int position, double motorPower) {
        arm.setTargetPosition(position);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorPower = motorPower;
        TotalTicks = position;
        StartingPosition = getCurrentPosition();
    }

    public void setPower(double motorPower) {
        arm.setPower(motorPower);
    }

    public boolean isBusy() {
        return arm.isBusy();
    }

    public int getCurrentPosition() {
        return arm.getCurrentPosition();
    }

    public int getTargetPosition() {
        return arm.getTargetPosition();
    }
}
