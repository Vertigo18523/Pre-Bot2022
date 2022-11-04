package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Base.BaseOpMode;
import org.firstinspires.ftc.teamcode.Base.Robot;
import org.firstinspires.ftc.teamcode.Bots.PreBot;
import org.firstinspires.ftc.teamcode.Utils.Action;

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
    public void onStart() throws InterruptedException {
        new Thread(
            new Action(() -> {
                robot.arm.move(robot.arm.HIGH_JUNCTION);
            }, true)
        ).start();
        robot.mecanum.driveForward(24);

    }
}