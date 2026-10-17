package org.firstinspires.ftc.teamcode.opmodes.teleop;


import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.helpers.Alliance;
import org.firstinspires.ftc.teamcode.MOEbot;
import org.firstinspires.ftc.teamcode.helpers.VelocityCompensationCalculator;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;

@NextTeleop(name = "Tele")
public class Tele extends NextOpMode {
    private final MOEbot robot;

    private final Pose startPose = new Pose(72, 72, Math.toRadians(180));
    private final double lastTurretTicksAtEndOfAuto = -999999;

    public Tele(MOEbot robot) {
        super(robot);
        this.robot = robot;

        robot.getFollower().setPose(startPose);
        Alliance.color = Alliance.BLUE;



        //setPoses();

    }
    //default startPose
    //length: 15.28
    //width: 15.12

    // Init loop
    @Override
    public void disabledPeriodic(){

    }


    // Run once, after play button clicked
    @Override
    public void start() {
        robot.startDrive(gamepad1);
        robot.getFlywheel().activate();
        //setPoses();

        //setColor();
    }

    //void setPoses();

    //void setColor();

    @Override
    public void periodic() {
        robot.getFollower().update();
        Telemetry.log("gamepady", gamepad1.left_stick_y);
    }
}