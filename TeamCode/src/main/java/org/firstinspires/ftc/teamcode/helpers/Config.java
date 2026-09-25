package org.firstinspires.ftc.teamcode.helpers;

import dev.nextftc.hardware.actuators.NextMotor.Direction;

public class Config {
    // Settings

    // region Control Hub

    // region leftFront
    public static String frontLeftName = "leftFront";
    public static int frontLeftPort = 0;
    public static Direction frontLeftDirection = Direction.FORWARD;

    // endregion

    // region rightFront
    public static String frontRightName = "rightFront";
    public static int frontRightPort = 1;
    public static Direction frontRightDirection = Direction.REVERSE;
    // endregion

    // region leftRear
    public static String backLeftName = "leftRear";
    public static int backLeftPort = 2;
    public static Direction backLeftDirection = Direction.FORWARD;
    // endregion

    // region rightRear
    public static String backRightName = "rightRear";
    public static int backRightPort = 3;
    public static Direction backRightDirection = Direction.REVERSE;
    // endregion

    // endregion

    // region Expansion Hub
    public static int intakeMotor = 0;

    public static String turretServoFName = "turretLeft";
    public static int turretServoF = 1;
    public static Direction turretServoFDirection = Direction.FORWARD;

    public static String turretServoBName = "turretRight";
    public static int turretServoB = 2;
    public static Direction turretServoBDirection = Direction.FORWARD;

    // endregion
}
