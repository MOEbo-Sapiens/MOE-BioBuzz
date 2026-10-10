package org.firstinspires.ftc.teamcode.mecanisms;

import static dev.nextftc.units.Units.RadiansPerSecond;
import static dev.nextftc.units.Units.Rotations;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.controllers.Controller;
import com.pedropathing.controllers.PIDController;
import com.qualcomm.hardware.lynx.LynxVoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.MOEbot;
import org.firstinspires.ftc.teamcode.helpers.RobotConfig;
import org.firstinspires.ftc.teamcode.helpers.VelocityCompensationCalculator;

import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;
import dev.nextftc.units.Measure;
import dev.nextftc.units.Unit;
import dev.nextftc.units.Units;
import dev.nextftc.units.measuretypes.Angle;
import dev.nextftc.units.measuretypes.AngularVelocity;
import dev.nextftc.units.measuretypes.Current;
import dev.nextftc.units.unittypes.AngularVelocityUnit;

@Config
public class Flywheel implements Mechanism {
    private NextMotor shooterMotorL = new NextMotor("leftFlywheel");
    private NextMotor shooterMotorR = new NextMotor("rightFlywheel");

    public static double kV = 0.000375, kP = 0.003, kS = 0.08; //TODO: TUNE VALUES

    private AngularVelocity target = RadiansPerSecond.of(0);
    private boolean activated = true;
    private double voltage = 12;

    public Flywheel() {
        shooterMotorR.follow(shooterMotorL, NextMotor.Direction.REVERSE);

        shooterMotorL.setDirection(NextMotor.Direction.FORWARD);

        shooterMotorL.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.FLOAT);

        shooterMotorL.getVelocityConstants().setKV(kV);
        shooterMotorL.getVelocityConstants().setKP(kP);
        shooterMotorL.getVelocityConstants().setKS(kS);

        shooterMotorL.setAnglePerCount(Rotations.of(28));
    }


    public AngularVelocity getCurrentAngularVel() {
        return shooterMotorL.getEncoderVelocity();
    }

    public void setTargetAngularVelocity(AngularVelocity target) {
        this.target = target;
    }


    public AngularVelocity getTargetAngularVelocity() {
        return target;
    }

    public void setPower(AngularVelocity power) {
        shooterMotorL.setVelocitySetpoint(power);
    }

    public void deactivate() {
        activated = false;
        setPower(new AngularVelocity(0, RadiansPerSecond));
    }

    public void activate() {
        activated = true;
    }

    public boolean isReady() {
        return getTargetAngularVelocity().isNear(getCurrentAngularVel(), 40);
    }

    public boolean getActivated() {
        return activated;
    }

    public Current getCurrent() {
        return shooterMotorL.getCurrent().plus(shooterMotorR.getCurrent());
    }

    public void toggle() {
        activated = !activated;
        if (!activated) {
            setPower(RadiansPerSecond.of(0));
        }
    }

    public void updateVoltage(double voltage) {
        this.voltage = voltage;
    }

    @Override
    public void periodic() {
        if (activated) {
            setTargetAngularVelocity(VelocityCompensationCalculator.getFlywheelRotationsPerSecond());
        }
    }
}