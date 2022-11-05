package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Base.BaseOpMode;
import org.firstinspires.ftc.teamcode.Base.Robot;
import org.firstinspires.ftc.teamcode.Bots.PreBot;
import org.firstinspires.ftc.teamcode.Components.Camera;

@Autonomous
public class Park extends BaseOpMode {
    public PreBot robot;

    Camera.ParkingPosition parkingPosition;

    @Override
    protected Robot setRobot() {
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
        robot.grabber.close();
    }

    @Override
    public void onStart() throws InterruptedException {
        parkingPosition = robot.camera.helper();
        robot.mecanum.strafeLeft(4);

        robot.mecanum.driveForward(28);
        if (parkingPosition == Camera.ParkingPosition.LEFT) {
            robot.mecanum.strafeLeft(24);
        } else if (parkingPosition == Camera.ParkingPosition.RIGHT) {
            robot.mecanum.strafeRight(24);
        }
    }
}
