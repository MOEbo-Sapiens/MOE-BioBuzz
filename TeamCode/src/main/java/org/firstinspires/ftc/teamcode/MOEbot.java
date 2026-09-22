package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.mecanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.robot.NextRobot;
import dev.nextftc.robot.drive.DriveCommands;

public class MOEbot implements NextRobot {
    public Follower follower;


    // Mechanisms
    private final Drivetrain drivetrain = new Drivetrain();

    public MOEbot(){}

    public Follower getFollower() {
        if (follower == null) {
            follower = Constants.create(RobotController.hardwareMap());
        }

        return follower;
    }

    public Drivetrain getDrivetrain() {
        return drivetrain;
    }
    public void startDrive(Gamepad gamepad) {
        drivetrain.startDrive(gamepad);
    }
}
