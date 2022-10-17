package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class TestAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        AutoBase robot = new AutoBase(
                this,
                hardwareMap,
                "frontLeft",
                "frontRight",
                "backLeft",
                "backRight",
                telemetry,
                0.25,
                0.25,
                14,
                13.5,
                1.13,
                50
        );
        waitForStart();
        if (opModeIsActive()) {
            robot.driveForward(24);
            robot.strafeLeft(24);
            robot.driveBackward(24);
            robot.strafeRight(24);
            robot.turnLeft();
            robot.strafeRight(24);
            robot.driveForward(24);
            robot.strafeLeft(24);
            robot.driveBackward(24);
            robot.turnRight();
        }
    }
}
