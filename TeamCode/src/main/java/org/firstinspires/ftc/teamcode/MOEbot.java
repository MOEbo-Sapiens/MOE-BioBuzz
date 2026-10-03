package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.helpers.HivePoses;
import org.firstinspires.ftc.teamcode.helpers.VelocityCompensationCalculator;
import org.firstinspires.ftc.teamcode.mecanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mecanisms.Hood;
import org.firstinspires.ftc.teamcode.mecanisms.Turret;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;

import java.util.Set;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;

public class MOEbot implements NextRobot {
    public Follower follower;


    // Mechanisms
    private final Drivetrain drivetrain = new Drivetrain();
    private final Hood hood = new Hood();
    private final Turret turret = new Turret();

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
    public Hood getHood() {
        return hood;
    }
    public Turret getTurret() {
        return turret;
    }
    public void startDrive(Gamepad gamepad) {
        drivetrain.startDrive(gamepad);
    }
    public void periodic() {
        HivePoses.update(getFollower());
        VelocityCompensationCalculator.update(getFollower());

    }

    @NonNull
    @Override
    public Set<Mechanism> getMechanisms() {
        return Set.of(drivetrain, hood, turret);
    }
}
