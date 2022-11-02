package org.firstinspires.ftc.teamcode.Base;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class BaseOpMode extends LinearOpMode {
    private Robot robot;

    public void runOpMode() throws InterruptedException {
        robot.initBot(hardwareMap, telemetry, this);
        onInit();
        robot.components.forEach(Component::init);

        waitForStart();
        onStart();
        robot.components.forEach(Component::start);

        while (opModeIsActive()) {
            onUpdate();
            robot.components.forEach(Component::update);
        }
    }

    public void onInit() {}
    public void onStart() {}
    public void onUpdate() {}
}