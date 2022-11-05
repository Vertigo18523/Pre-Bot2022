package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Base.BaseOpMode;
import org.firstinspires.ftc.teamcode.Base.Robot;
import org.firstinspires.ftc.teamcode.Bots.PreBot;
import org.firstinspires.ftc.teamcode.Components.Camera;

@Autonomous
public class Left_Deliver1Far_SensePark extends BaseOpMode {
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

        // move to junction pole
        robot.arm.move(robot.arm.HIGH_JUNCTION);
        robot.mecanum.driveForward(6);
        robot.mecanum.strafeRight(24);
        robot.mecanum.driveForward(60);
        robot.mecanum.turnLeft();

        // deliver preload cone
        robot.mecanum.driveForward(12);
        robot.grabber.open();
        robot.mecanum.driveBackward(12);

        // line up for parking
        robot.mecanum.strafeLeft(36); // or 12 to be in the upper half of the signal zone but not have to deal with the signal
        robot.arm.move(robot.arm.ZERO_POSITION);

        // park
        if (parkingPosition == Camera.ParkingPosition.LEFT) {
            robot.mecanum.driveForward(48);
        } else if (parkingPosition == Camera.ParkingPosition.CENTER) {
            robot.mecanum.driveForward(24);
        }
        robot.mecanum.turnRight();
    }
}
