package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class TestAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AutoBase robot = new AutoBase(
                this,
                hardwareMap,
                "frontLeft",
                "frontRight",
                "backLeft",
                "backRight",
                telemetry,
                0.1,
                0.5,
                10.5,
                12.5,
                1.1,
                1000,
                false,
                0,
                0,
                0
        );
        waitForStart();
        if (opModeIsActive()) {
            robot.driveForward(24);
        }
    }
}
