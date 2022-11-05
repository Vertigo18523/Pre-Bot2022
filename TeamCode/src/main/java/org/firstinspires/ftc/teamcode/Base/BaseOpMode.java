package org.firstinspires.ftc.teamcode.Base;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Utils.Action;

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
//        new Action(() -> {
//            try {
//                onStart();
//                robot.components.forEach(Component::start);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//        }, true).run();
        onStart();
        robot.components.forEach(Component::start);

//        new Action(() -> {
//            try {
//                while (opModeIsActive()) {
//                    onUpdate();
//                    robot.components.forEach(Component::update);
//                }
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//        }, true).run();
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