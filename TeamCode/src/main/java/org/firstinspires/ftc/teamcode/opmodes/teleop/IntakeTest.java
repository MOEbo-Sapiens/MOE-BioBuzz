package org.firstinspires.ftc.teamcode.opmodes.teleop;

import androidx.annotation.NonNull;

import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.MOEbot;
import org.firstinspires.ftc.teamcode.mecanisms.Intake;

import java.util.List;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.opmode.OpModeHook;
import dev.nextftc.robot.triggers.CommandGamepad;
import dev.nextftc.robot.triggers.Trigger;

@NextTeleop(name = "Intake Test")
public class IntakeTest extends NextOpMode {
    private final MOEbot robot;

    public IntakeTest(MOEbot robot) {
        super(robot);
        this.robot = robot;
        Scheduler.reset();
    }

    // Init loop
    @Override
    public void disabledPeriodic(){

    }

    // Run once, after play button clicked
    @Override
    public void start() {
        Trigger.Companion.getDefaultEventLoop().clear();
        CommandGamepad gp1 = new CommandGamepad(gamepad1);
        gp1.leftBumper().onTrue(robot.getIntake().runIntake(1));
        gp1.rightBumper().onTrue(robot.getIntake().runIntake(-1));
    }

    @Override
    public void periodic() {
        Telemetry.log("Intake Current", robot.getIntake().getCurrent().getAbsoluteValue());
        Telemetry.log("Beam Broken", robot.getIntake().isBeamBroken());
    }
}
