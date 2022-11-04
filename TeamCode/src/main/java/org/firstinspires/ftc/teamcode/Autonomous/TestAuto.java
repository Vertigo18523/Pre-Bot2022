package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Base.BaseOpMode;
import org.firstinspires.ftc.teamcode.Base.Robot;
import org.firstinspires.ftc.teamcode.Bots.PreBot;

@Autonomous
public class TestAuto extends BaseOpMode {
    public PreBot robot;

    @Override
    public Robot setRobot() {
        this.robot = new PreBot();
        return this.robot;
    }

    @Override
    protected boolean setTeleOp() {
        return false;
    }

    @Override
    public void onInit() throws InterruptedException {
        robot.camera.requestStart();
    }

    @Override
    public void onStart() throws InterruptedException {
        robot.camera.helper();
        robot.mecanum.driveForward(24);
    }
}
