package org.firstinspires.ftc.teamcode.opmodes.utility;


import static dev.nextftc.units.Units.Rotations;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.helpers.Alliance;
import org.firstinspires.ftc.teamcode.MOEbot;
import org.firstinspires.ftc.teamcode.helpers.VelocityCompensationCalculator;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Telemetry;

@Utility(name = "Flywheel Tuner")
@Config
public class FlywheelTuner extends OpMode {
    public static double kS = 0.09, kV = 0.000059, kP = 0.004;
    private final VoltageSensor voltageSensor;
    private final NextMotor shooterMotorL = new NextMotor("leftFlywheel");
    private final NextMotor shooterMotorR = new NextMotor("rightFlywheel");
    public static double targetTicks = 0;
    private static double currentTicks = 0;
    public static double error = 0;
    private double power = 0;

    public FlywheelTuner() {
        shooterMotorL.setDirection(NextMotor.Direction.FORWARD);
        shooterMotorL.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.FLOAT);
        shooterMotorR.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.FLOAT);
        shooterMotorR.follow(shooterMotorL, NextMotor.Direction.REVERSE);

        shooterMotorL.setAnglePerCount(Rotations.of(1));

        voltageSensor = RobotController.hardwareMap().get(VoltageSensor.class, "Control Hub");
    }

    public void setPower(double power) {
        shooterMotorL.setThrottle(power);
    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        currentTicks = shooterMotorL.getEncoderVelocity().getBaseUnitMagnitude();
        error = targetTicks - currentTicks;

        if (targetTicks < 10) {
            kS = 0;
        }

        power =
                (kV * targetTicks)
                        + (kP * (targetTicks - currentTicks))
                        + kS;

        double batteryVoltage = voltageSensor.getVoltage();
        power *= (12.0 / batteryVoltage);


        setPower(Math.max(Math.min(power, 1.0), -1.0));

        Telemetry.log("Current Flywheel Vel", currentTicks);
        Telemetry.log("Target Flywheel Vel", targetTicks);
        Telemetry.log("Voltage", voltageSensor.getVoltage());
        Telemetry.log("Error", error);
        Telemetry.log("Power", power);
        Telemetry.addBackend(FtcDashboard.getInstance().getTelemetry());
        Telemetry.update();
        shooterMotorL.update();
        shooterMotorR.update();
    }
}