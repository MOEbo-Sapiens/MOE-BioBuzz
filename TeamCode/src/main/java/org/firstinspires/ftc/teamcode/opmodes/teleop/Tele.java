package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static com.pedropathing.ivy.commands.Commands.infinite;


import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.helpers.Alliance;
import org.firstinspires.ftc.teamcode.helpers.HivePoses;
import org.firstinspires.ftc.teamcode.helpers.RobotConfig;
import org.firstinspires.ftc.teamcode.MOEbot;
import org.firstinspires.ftc.teamcode.mecanisms.Turret;

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
        Scheduler.reset();

    }
    //default startPose
    //length: 15.28
    //width: 15.12


    public void initialize() {
        robot.getFollower().setPose(startPose);
        Alliance.color = Alliance.BLUE;
        //setPoses();
        Scheduler.reset();

        //setColor();
    }

    //void setPoses();

    //void setColor();

    @Override
    public void periodic() {
        Scheduler.execute();
        robot.getFollower().update();
    }




}