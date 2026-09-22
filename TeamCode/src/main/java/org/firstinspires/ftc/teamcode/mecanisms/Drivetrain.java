package org.firstinspires.ftc.teamcode.mecanisms;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.helpers.Config;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.drive.DriveCommands;

public class Drivetrain implements Mechanism {
    public Drivetrain() {
        frontLeft.setDirection(Config.frontLeftDirection);
        frontRight.setDirection(Config.frontRightDirection);
        backLeft.setDirection(Config.backLeftDirection);
        backRight.setDirection(Config.backRightDirection);
    }

    public final NextMotor frontLeft = new NextMotor(RobotController.controlHub(), Config.frontLeftPort);
    public final NextMotor frontRight = new NextMotor(RobotController.controlHub(), Config.frontRightPort);
    public final NextMotor backLeft = new NextMotor(RobotController.controlHub(), Config.backLeftPort);
    public final NextMotor backRight = new NextMotor(RobotController.controlHub(), Config.backRightPort);

    public void startDrive(Gamepad gamepad) {
        DriveCommands.mecanumDrive(frontLeft, frontRight, backLeft, backRight, gamepad).schedule();
    }
}
