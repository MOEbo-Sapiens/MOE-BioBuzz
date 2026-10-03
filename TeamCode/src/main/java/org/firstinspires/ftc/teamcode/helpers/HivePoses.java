package org.firstinspires.ftc.teamcode.helpers;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

public class HivePoses {
    public static Pose RED_HIVE_POSE_LOWER = new Pose(58, 57, Math.toRadians(0));
    public static Pose RED_HIVE_POSE_UPPER = new Pose(58, 83, Math.toRadians(0));
    public static Pose BLUE_HIVE_POSE_LOWER = new Pose(83.5, 57, Math.toRadians(0));
    public static Pose BLUE_HIVE_POSE_UPPER = new Pose(83.5, 83, Math.toRadians(0));

    public static Pose currentHive = BLUE_HIVE_POSE_LOWER;

    public static void update(Follower follower) {
        if (Alliance.color == Alliance.RED) {
            if (follower.pose().y() > 72) {
                currentHive = RED_HIVE_POSE_UPPER;
            } else {
                currentHive = RED_HIVE_POSE_LOWER;
            }
        } else {
            if (follower.pose().y() > 72) {
                currentHive = BLUE_HIVE_POSE_UPPER;
            } else {
                currentHive = BLUE_HIVE_POSE_LOWER;
            }
        }
    }
    public static Pose getCurrentHivePose() {
        return currentHive;
    }
}
