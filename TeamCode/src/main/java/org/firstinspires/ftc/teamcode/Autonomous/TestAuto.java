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
    public void onStart() throws InterruptedException {
        //almost working
        Runnable r = new MoveArm(robot);
        new Thread(r).start();
        robot.mecanum.driveForward(24);

}





}
//Runnable instead of Thread lets us pass parameters
class MoveArm implements Runnable {
    PreBot robot;

    public MoveArm(PreBot robot) {
        this.robot = robot;

    }

    public void run()
    {
        robot.arm.move(robot.arm.HIGH_JUNCTION);
    }
}