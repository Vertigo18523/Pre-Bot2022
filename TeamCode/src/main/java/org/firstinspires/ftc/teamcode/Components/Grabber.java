package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.Component;

public class Grabber implements Component {
    private final Servo grabber;

    private final Telemetry telemetry;

    public double OPEN;
    public double CLOSED;

    public boolean isGrabbing = false;
    public boolean changed = false;

    public Grabber(String deviceName, HardwareMap hardwareMap, Telemetry telemetry) {
        grabber = hardwareMap.get(Servo.class, deviceName);
        this.OPEN = 0;
        this.CLOSED = 1;
        this.telemetry = telemetry;
    }

    @Override
    public void init() {
        grabber.setPosition(OPEN);
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        grabber.setPosition(isGrabbing ? CLOSED : OPEN);
    }

    public void buttonPressed() {
        if (!changed) {
            isGrabbing = !isGrabbing;
            changed = true;
        }
    }

    public void buttonReleased() {
        changed = false;
    }
}
