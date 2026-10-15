package org.firstinspires.ftc.teamcode.helpers.drivetrains;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.utils.Angle;
import com.pedropathing.utils.Utils;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.helpers.Alliance;
import org.firstinspires.ftc.teamcode.helpers.math.MathHelpers;

import java.util.function.Supplier;

import dev.nextftc.control.feedback.PIDController;
import dev.nextftc.hardware.actuators.NextMotor;
import smile.math.MathEx;


public class MecanumHeadingLock {

    private static final double HEADING_POWER_MIN = -1.0;
    private static final double HEADING_POWER_MAX = 1.0;

    public static double ROTATE_STICK_THRESHOLD = 0.025;
    public static double ANGULAR_VELOCITY_THRESHOLD = 0.2;

    private final PIDController anglePID = new PIDController(5.752843168584152);

    private final PIDController headingPID =
            new PIDController(anglePID.getCoefficients());

    private boolean headingLockActive = false;
    private double targetHeading = 0.0;

    public static Command MecanumHeadingLock(NextMotor frontLeft, NextMotor frontRight, NextMotor backLeft, NextMotor backRight,
                                             Gamepad gamepad,
                                             Follower follower) {
        MecanumHeadingLock mecanumHeadingLock = new MecanumHeadingLock();
        return Commands.infinite(() -> {
            mecanumHeadingLock.arcade(gamepad.left_stick_y, gamepad.left_stick_x, gamepad.right_stick_x, follower);
        });
    }

    public void arcade(double forward, double strafe, double rotateX, Follower follower) {
        //Can also look to fix the heading lock PID, but this is a simpler fix for now.

        double theta = 0;
        double heading = follower.pose().heading();

        if (Alliance.color == Alliance.RED) {
            theta = -heading;
        } else if (Alliance.color == Alliance.BLUE) {
            theta = MathHelpers.wrapAngleRadians(-heading + Math.toRadians(180));
        } /*else if (Alliance.color == Alliance.AUDIENCE) {
                theta = MathHelpers.wrapAngleRadians(-heading + Math.toRadians(90));
            }*/

        double cos = Math.cos(theta);
        double sin = Math.sin(theta);
        double strafeRot = strafe * cos - forward * sin;
        double forwardRot = strafe * sin + forward * cos;
        strafe = strafeRot;
        forward = forwardRot;


        double angularVelocityRadPerSec = follower.velocity().omega;

        boolean rotateStickActive = Math.abs(rotateX) >= ROTATE_STICK_THRESHOLD;

        if (rotateStickActive) {
            headingLockActive = false;
        } else if (!headingLockActive
                && Math.abs(angularVelocityRadPerSec) < ANGULAR_VELOCITY_THRESHOLD) {
            headingLockActive = true;
            targetHeading = heading;
        }

        double headingPower = rotateX;
        if (headingLockActive) {
            headingPower = -calculateHeadingPower(targetHeading, heading);
        }


    }

    private double calculateHeadingPower(double targetHeading, double currentHeading) {
        double headingError = Angle.turnDirection(currentHeading, targetHeading)
                * Angle.smallestDifference(currentHeading, targetHeading);


        return Utils.clamp(
                headingPID.calculate(headingError),
                HEADING_POWER_MIN,
                HEADING_POWER_MAX);
    }

    public void setTargetHeading(double radians) {
        targetHeading = radians;
        headingLockActive = true;
    }
}