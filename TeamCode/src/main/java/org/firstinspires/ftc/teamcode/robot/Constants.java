package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.shooter.Turret;

@Config
public class Constants {
    public static Pose BLUE_GOAL_POSE = new Pose(3, 133.5, Math.toRadians(0));
    public static Pose RED_GOAL_POSE = new Pose(133.5, 136.5, Math.toRadians(0));

    public static Pose RED_HIVE_POSE_LOWER = new Pose(58, 57, Math.toRadians(0));
    public static Pose RED_HIVE_POSE_UPPER = new Pose(58, 83, Math.toRadians(0));

    // red perp pollen right side x 8.61 y 36.924

    public enum Color {
        BLUE,
        RED,
        AUDIENCE
    }

    public static Color color = Color.BLUE;

//    public static Robot robot;
    public static Pose lastPose = null;

    public static void reset() {
        Turret.turretOffsetRad = 0;
//        robot = null;
        lastPose = null;
    }

    public static boolean lastOpModeWasAuto = false;
    public static boolean debugTelemetry = false;

    // thresholds for caching wrappers
    public static double MOTOR_CACHING_TOLERANCE = 0.05;
    public static double CRSERVO_CACHING_TOLERANCE = 0.05;
    public static double SERVO_CACHING_TOLERANCE = 0.01;

    public static double PROXIMITY_POLL_MS = 100.0;
    public static double PROXIMITY_POLL_MS_FULL = 300.0;
    public static double TELEMETRY_UPDATE_MS = 125.0;
}
