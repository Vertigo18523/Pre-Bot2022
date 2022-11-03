package org.firstinspires.ftc.teamcode.Base;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class BaseOpMode extends LinearOpMode {
    private Robot robot;
    private boolean isTeleOp;

    public void runOpMode() throws InterruptedException {
        robot = setRobot();
        isTeleOp = setTeleOp();

        robot.initBot(hardwareMap, telemetry, this, isTeleOp);

        robot.components.forEach(Component::init);
        onInit();

        waitForStart();
        onStart();
        robot.components.forEach(Component::start);

        while (opModeIsActive()) {
            onUpdate();
            robot.components.forEach(Component::update);
        }
    }

    public void onInit() throws InterruptedException {
    }

    public void onStart() throws InterruptedException {
    }

    public void onUpdate() throws InterruptedException {
    }

    protected abstract Robot setRobot();
    protected abstract boolean setTeleOp();
}