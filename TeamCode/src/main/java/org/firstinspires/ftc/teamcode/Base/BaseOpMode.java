package org.firstinspires.ftc.teamcode.Base;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Bots.PreBot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public abstract class BaseOpMode extends LinearOpMode {
    private Robot robot;

    public void runOpMode() throws InterruptedException {
        robot = setRobot();
        robot.initBot(hardwareMap, telemetry, this);

        robot.components.forEach(Component::init);
        onInit();

        waitForStart();
        robot.components.forEach(Component::start);
        onStart();

        while (opModeIsActive()) {
            robot.components.forEach(Component::update);
            onUpdate();
        }
    }

    public void onInit() throws InterruptedException {}
    public void onStart() throws InterruptedException {}
    public void onUpdate() throws InterruptedException {}

    protected abstract Robot setRobot();
}