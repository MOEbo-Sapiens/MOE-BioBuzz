package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.helpers.HivePoses;
import org.firstinspires.ftc.teamcode.helpers.VelocityCompensationCalculator;
import org.firstinspires.ftc.teamcode.mecanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mecanisms.Flywheel;
import org.firstinspires.ftc.teamcode.mecanisms.GateServo;
import org.firstinspires.ftc.teamcode.mecanisms.Hood;
import org.firstinspires.ftc.teamcode.mecanisms.Turret;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;

import java.util.Set;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;
import dev.nextftc.robot.Telemetry;

public class MOEbot implements NextRobot {
    public Follower follower;
    //public final VoltageSensor voltageSensor;


    // Mechanisms
    private final Drivetrain drivetrain = new Drivetrain();
    private final Hood hood = new Hood();
    private final Turret turret = new Turret();
    private final Flywheel flywheel = new Flywheel();
    private final GateServo gateServo = new GateServo();

    public MOEbot (){
        //voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

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
    public Flywheel getFlywheel() {
        return flywheel;
    }
    public GateServo getGateServo() {
        return gateServo;
    }

    public void startDrive(Gamepad gamepad) {
        drivetrain.startDrive(gamepad, getFollower());
    }

    @Override
    public void periodic() {
        HivePoses.update(getFollower());
        VelocityCompensationCalculator.update(getFollower());
        getDrivetrain().updatePolygonPoses(getFollower());
        //getFlywheel().updateVoltage(voltageSensor.getVoltage());
        getFollower().update();
    }

    @NonNull
    @Override
    public Set<Mechanism> getMechanisms() {
        return Set.of(
                drivetrain,
                hood,
                turret,
                flywheel,
                gateServo
        );
    }
}
