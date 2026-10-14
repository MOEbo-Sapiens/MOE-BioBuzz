package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.robot.Robot;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor.Direction;
import dev.nextftc.hardware.lynx.NextLynxModule;

public class RobotConfig {
    // Settings
    public static double robotWidth = 18;
    public static double robotLength = 18;

    public static boolean lastOpModeWasAuto = false;



    // region Control Hub

    // region leftFront
    public static NextLynxModule frontLeftHub = RobotController.expansionHub();
    public static String frontLeftName = "leftFront";
    public static int frontLeftPort = 1;
    public static Direction frontLeftDirection = Direction.FORWARD;

    // endregion

    // region rightFront
    public static NextLynxModule frontRightHub = RobotController.expansionHub();
    public static String frontRightName = "rightFront";
    public static int frontRightPort = 0;
    public static Direction frontRightDirection = Direction.REVERSE;
    // endregion

    // region leftRear
    public static NextLynxModule backLeftHub = RobotController.controlHub();
    public static String backLeftName = "leftRear";
    public static int backLeftPort = 1;
    public static Direction backLeftDirection = Direction.FORWARD;
    // endregion

    // region rightRear
    public static NextLynxModule backRightHub = RobotController.controlHub();
    public static String backRightName = "rightRear";
    public static int backRightPort = 0;
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
