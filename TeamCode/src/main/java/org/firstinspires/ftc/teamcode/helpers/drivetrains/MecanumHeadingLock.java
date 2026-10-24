package org.firstinspires.ftc.teamcode.helpers.drivetrains;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.utils.Angle;
import com.pedropathing.utils.Utils;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

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

    private final PIDController anglePID = new PIDController(1.5, 0, 0.2);

    private final PIDController headingPID =
            new PIDController(anglePID.getCoefficients());

    private boolean headingLockActive = false;
    private double targetHeading = 0.0;

    private NextMotor frontLeft, frontRight, backLeft, backRight;

    public MecanumHeadingLock(NextMotor frontLeft, NextMotor frontRight, NextMotor backLeft, NextMotor backRight){
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
    }

    public void arcade(double forward, double strafe, double rotateX, Follower follower) {
        //Can also look to fix the heading lock PID, but this is a simpler fix for now.

        double theta = 0;
        double heading = follower.pose().heading();


//        if (Alliance.color == Alliance.RED) {
//            theta = -heading;
//        } else if (Alliance.color == Alliance.BLUE) {
//            theta = MathHelpers.wrapAngleRadians(-heading + Math.toRadians(180));
//        }
//
//        double cos = Math.cos(theta);
//        double sin = Math.sin(theta);
//        double strafeRot = strafe * cos - forward * sin;
//        double forwardRot = strafe * sin + forward * cos;
//        strafe = strafeRot;
//        forward = forwardRot;


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

        mecanumPower(forward, strafe, headingPower, 1.0, 1.0);


    }

    public Command headingLock(Gamepad gamepad, Follower follower) {
        return Commands.infinite(() -> {
            arcade(-gamepad.left_stick_y, gamepad.left_stick_x, gamepad.right_stick_x, follower);
        });
    }

    public void mecanumPower(double forward, double strafe, double rotateX, double speed, double rotSpeed) {

        rotateX *= rotSpeed;
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotateX), 1);
        double frontLeftPower = (forward + strafe + rotateX) / denominator;
        double backLeftPower = (forward - strafe + rotateX) / denominator;
        double frontRightPower = (forward - strafe - rotateX) / denominator;
        double backRightPower = (forward + strafe - rotateX) / denominator;


        frontLeft.setThrottle(frontLeftPower);
        backLeft.setThrottle(backLeftPower);
        frontRight.setThrottle(frontRightPower);
        backRight.setThrottle(backRightPower);
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