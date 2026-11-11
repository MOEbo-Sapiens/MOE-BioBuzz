package org.firstinspires.ftc.teamcode.mecanisms;

import static dev.nextftc.units.Units.Radians;
import static dev.nextftc.units.Units.RadiansPerSecond;
import static dev.nextftc.units.Units.Rotations;
import static dev.nextftc.units.Units.RotationsPerSecond;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.controllers.Controller;
import com.pedropathing.controllers.PIDController;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.hardware.lynx.LynxVoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.MOEbot;
import org.firstinspires.ftc.teamcode.helpers.RobotConfig;
import org.firstinspires.ftc.teamcode.helpers.VelocityCompensationCalculator;
import org.firstinspires.ftc.teamcode.opmodes.teleop.Tele;

import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;
import dev.nextftc.robot.Telemetry;
import dev.nextftc.units.Measure;
import dev.nextftc.units.Unit;
import dev.nextftc.units.Units;
import dev.nextftc.units.measuretypes.Angle;
import dev.nextftc.units.measuretypes.AngularVelocity;
import dev.nextftc.units.measuretypes.Current;
import dev.nextftc.units.unittypes.AngularVelocityUnit;

@Config
public class Flywheel implements Mechanism {
    private final NextMotor shooterMotorL = new NextMotor("leftFlywheel");
    private final NextMotor shooterMotorR = new NextMotor("rightFlywheel");
    private final VoltageSensor voltageSensor;

    public static double kS = 0.09, kV = 0.000059, kP = 0.004;
    private boolean activated = true;

    private double power = 0.0;
    private double error = 0.0;

    public Flywheel() {
        shooterMotorL.setDirection(NextMotor.Direction.FORWARD);
        shooterMotorL.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.FLOAT);
        shooterMotorR.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.FLOAT);
        shooterMotorR.follow(shooterMotorL, NextMotor.Direction.REVERSE);

        shooterMotorL.setAnglePerCount(Rotations.of(1));

        voltageSensor = RobotController.hardwareMap().get(VoltageSensor.class, "Control Hub");
    }


    public AngularVelocity getCurrentAngularVel() {
        return shooterMotorL.getEncoderVelocity();
    }

    public double getDoubleCurrentAngularVelocity() {
        return getCurrentAngularVel().getMagnitude();
    }

    public void setTargetAngularVelocity(AngularVelocity target) {
        double targetVelocity = getTargetAngularVelocity().getMagnitude();
        double measuredVelocity = getCurrentAngularVel().getMagnitude();

        error = targetVelocity - measuredVelocity;

        if (getTargetAngularVelocity().isNear(RotationsPerSecond.of(0), 0.05)) {
            kS = 0;
        }

        power =
                (kV * targetVelocity)
                        + (kP * (targetVelocity - measuredVelocity))
                        + kS;

        double batteryVoltage = voltageSensor.getVoltage();
        power *= (12.0 / batteryVoltage);

        setPower(Math.max(Math.min(power, 1.0), -1.0));
    }


    public AngularVelocity getTargetAngularVelocity() {
        return VelocityCompensationCalculator.getFlywheelRotationsPerSecond();
    }

    public void setPower(double power) {
        shooterMotorL.setThrottle(power);
    }

    public void deactivate() {
        activated = false;
        setPower(0);
    }

    public Command activate() {
        return infinite(() -> setTargetAngularVelocity(VelocityCompensationCalculator.getFlywheelRotationsPerSecond()));
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
            setPower(0);
        }
    }

    @Override
    public void periodic() {
        Telemetry.log("Current Flywheel Vel", getCurrentAngularVel().getMagnitude());
        Telemetry.log("Target Flywheel Vel", getTargetAngularVelocity().getMagnitude());


        Telemetry.log("Voltage", voltageSensor.getVoltage());

        Telemetry.log("Ticks/Second Target", getTargetAngularVelocity());
        Telemetry.log("Ticks/Second Actual", shooterMotorL.getEncoderVelocity());

        Telemetry.log("Flywheel Throttle", power);
        Telemetry.log("Flywheel Velocity Error", error);
    }

}