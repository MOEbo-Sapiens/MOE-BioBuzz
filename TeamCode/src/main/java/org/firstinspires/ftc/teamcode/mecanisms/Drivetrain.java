package org.firstinspires.ftc.teamcode.mecanisms;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

import org.firstinspires.ftc.teamcode.MOEbot;
import org.firstinspires.ftc.teamcode.helpers.Alliance;
import org.firstinspires.ftc.teamcode.helpers.RobotConfig;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;
import dev.nextftc.robot.drive.DriveCommands;

public class Drivetrain implements Mechanism {
    private final PolygonZone robotZone = new PolygonZone(RobotConfig.robotWidth, RobotConfig.robotLength);

    private final PolygonZone redLowerZone = new PolygonZone(new Point(0,0), new Point(144, 0), new Point(144, 31.2), new Point(59.44, 40.27), new Point(0, 31.57));
    private final PolygonZone blueLowerZone = new PolygonZone(new Point(144, 0), new Point(0, 0), new Point(0, 31.2), new Point(84.56, 40.27), new Point(144, 31.57));
    private final PolygonZone redUpperZone = new PolygonZone(new Point(0, 144), new Point(144, 144), new Point(144, 112.8), new Point(59.44, 103.73), new Point(0, 112.43));
    private final PolygonZone blueUpperZone = new PolygonZone(new Point(144, 144), new Point(0, 144), new Point(0, 112.8), new Point(84.56, 103.73), new Point(144, 112.43));

    public Drivetrain() {
        frontLeft.setDirection(RobotConfig.frontLeftDirection);
        frontRight.setDirection(RobotConfig.frontRightDirection);
        backLeft.setDirection(RobotConfig.backLeftDirection);
        backRight.setDirection(RobotConfig.backRightDirection);
    }

    public final NextMotor frontLeft = new NextMotor(RobotController.controlHub(), RobotConfig.frontLeftPort);
    public final NextMotor frontRight = new NextMotor(RobotController.controlHub(), RobotConfig.frontRightPort);
    public final NextMotor backLeft = new NextMotor(RobotController.controlHub(), RobotConfig.backLeftPort);
    public final NextMotor backRight = new NextMotor(RobotController.controlHub(), RobotConfig.backRightPort);

    public void startDrive(Gamepad gamepad) {
        DriveCommands.mecanumDrive(frontLeft, frontRight, backLeft, backRight, gamepad).schedule();
    }

    public void updatePolygonPoses(Follower follower) {
        robotZone.setPosition(72 - follower.pose().y(), follower.pose().x() - 72);
    }

    public boolean canShoot() {
        if (Alliance.color == Alliance.RED) {
            return robotZone.isFullyInside(redUpperZone) || robotZone.isFullyInside(redLowerZone);
        } else {
            return robotZone.isFullyInside(blueUpperZone) || robotZone.isFullyInside(blueLowerZone);
        }
    }
}
