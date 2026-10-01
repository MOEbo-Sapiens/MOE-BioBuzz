package org.firstinspires.ftc.teamcode.helpers;

public class MathHelpers {
    public static double wrapAngleRadians(double angle) {
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        while (angle > Math.PI) angle -= 2 * Math.PI;
        return angle;
    }

    public static double wrapAngleDegrees(double angle) {
        while (angle <= -180) angle += 360;
        while (angle > 180) angle -= 360;
        return angle;
    }
}