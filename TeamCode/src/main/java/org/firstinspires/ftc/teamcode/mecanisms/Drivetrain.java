package org.firstinspires.ftc.teamcode.mecanisms;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.helpers.RobotConfig;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.drive.DriveCommands;

public class Drivetrain implements Mechanism {
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
}
