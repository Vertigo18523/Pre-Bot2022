package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Base.BaseOpMode;
import org.firstinspires.ftc.teamcode.Base.Robot;
import org.firstinspires.ftc.teamcode.Bots.PreBot;
import org.firstinspires.ftc.teamcode.Components.Camera;

@Autonomous
public class Left_Deliver1Close_SensePark extends BaseOpMode {
    public PreBot robot;

    Camera.ParkingPosition parkingPosition;

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
        robot.grabber.close();
    }

    @Override
    public void onStart() throws InterruptedException {
        parkingPosition = robot.camera.helper();

        // move to junction pole
        robot.arm.move(robot.arm.HIGH_JUNCTION);
        robot.mecanum.driveForward(6);
        robot.mecanum.strafeRight(36);

        // deliver preload cone
        robot.mecanum.driveForward(32);
        robot.grabber.open();
        robot.mecanum.driveBackward(18);

        // line up for parking
        robot.mecanum.turnLeft();
        robot.arm.move(robot.arm.ZERO_POSITION);

        // park
        if (parkingPosition == Camera.ParkingPosition.LEFT) {
            robot.mecanum.driveForward(60);
        } else if (parkingPosition == Camera.ParkingPosition.CENTER) {
            robot.mecanum.driveForward(36);
        } else if (parkingPosition == Camera.ParkingPosition.RIGHT) {
            robot.mecanum.driveForward(12);
        }
        robot.mecanum.turnRight();
    }
}
