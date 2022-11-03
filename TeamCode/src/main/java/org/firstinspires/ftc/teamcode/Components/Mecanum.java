package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.Component;

public class Mecanum implements Component {
    public final DcMotor frontLeft;
    public final DcMotor backLeft;
    public final DcMotor frontRight;
    public final DcMotor backRight;

    public final Telemetry telemetry;

    public float x;
    public float y;
    public float clockwise;
    public double fl;
    public double fr;
    public double bl;
    public double br;
    public int speed = 1;
    public boolean slowModeChanged = false;
    public boolean shouldSlowMode = false;

    public Mecanum(String flName, String blName, String frName, String brName, HardwareMap hardwareMap, Telemetry telemetry) {
        frontLeft = hardwareMap.get(DcMotor.class, flName);
        backLeft = hardwareMap.get(DcMotor.class, blName);
        frontRight = hardwareMap.get(DcMotor.class, frName);
        backRight = hardwareMap.get(DcMotor.class, brName);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        this.telemetry = telemetry;
    }

    @Override
    public void init() {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void start() {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void update() {
        if (shouldSlowMode) {
            if (speed > 2) {
                setHalfSpeed();
            }
        }
        telemetry.addData("Speed", "1 / " + speed);
        frontLeft.setPower(fl / speed);
        frontRight.setPower(fr / speed);
        backLeft.setPower(bl / speed);
        backRight.setPower(br / speed);
    }

    public void driveForward() {
        y = (float) 1.0;
    }

    public void driveBackward() {
        y = (float) -1.0;
    }

    public void strafeRight() {
        x = (float) 1.0;
    }

    public void strafeLeft() {
        x = (float) -1.0;
    }

    public void turnRight() {
        clockwise = (float) 1.0;
    }

    public void turnLeft() {
        clockwise = (float) -1.0;
    }

    public void setInitialDirections(double y, double x, double clockwise) {
        fl = -y + x + clockwise;
        fr = -y - x - clockwise;
        bl = -y - x + clockwise;
        br = -y + x - clockwise;
    }

    public void setFullSpeed() {
        speed = 1;
    }

    public void setHalfSpeed() {
        speed = 2;
    }

    public void setQuarterSpeed() {
        speed = 4;
    }

    public void buttonPressed() {
        if (!slowModeChanged) {
            shouldSlowMode = !shouldSlowMode;
            slowModeChanged = true;
        }
    }

    public void buttonReleased() {
        if (slowModeChanged) {
            slowModeChanged = false;
        } else {
            setFullSpeed();
        }
    }
}
