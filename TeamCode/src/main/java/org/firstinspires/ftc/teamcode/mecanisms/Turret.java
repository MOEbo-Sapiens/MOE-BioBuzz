package org.firstinspires.ftc.teamcode.mecanisms;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.helpers.RobotConfig;
import org.firstinspires.ftc.teamcode.helpers.VelocityCompensationCalculator;

import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

public class Turret implements Mechanism {
    NextServo turretServoF = new NextServo(RobotConfig.turretServoFName, 0.0025, RobotConfig.turretServoFDirection);
    NextServo turretServoB = new NextServo(RobotConfig.turretServoBName, 0.0025, RobotConfig.turretServoBDirection);

    private boolean autoAimEnabled = true;

    public static double turretOffsetRad = 0;
    public static double MIN_TURRET_ANGLE = Math.toRadians(-157);
    public static double MAX_TURRET_ANGLE = Math.toRadians(157);

    double[] angleValues = new double[] {Math.toRadians(-90), Math.toRadians(90)};
    double[] servoPositions = new double[] {0.78, 0.22}; //TODO: tune


    public double getAngleFromServoPos(double servoPos) {
        return (servoPos - servoPositions[0]) *
                (angleValues[1] - angleValues[0])/(servoPositions[1] - servoPositions[0]) //slope
                + angleValues[0]; //intercept
    }

    public double getServoPosFromAngle(double angleRad) {
        return (angleRad - angleValues[0]) *
                (servoPositions[1] - servoPositions[0]) / (angleValues[1] - angleValues[0]) //slope
                + servoPositions[0]; //intercept
    }


    public double MIN_SERVO_POS = Math.min(getServoPosFromAngle(MIN_TURRET_ANGLE), getServoPosFromAngle(MAX_TURRET_ANGLE));
    public double MAX_SERVO_POS = Math.max(getServoPosFromAngle(MIN_TURRET_ANGLE), getServoPosFromAngle(MAX_TURRET_ANGLE));

    public double getTargetPosition() {
        return turretServoF.getPosition();
    }

    /**
     * @return The target angle of the turret in radians. Note that counter clockwise is positive with 0 being straight forward.
     */
    public double getTargetAngle() {
        double currentPos = getTargetPosition();
        return getAngleFromServoPos(currentPos);
    }

    public double getCurrentAngle() {
        return getAngleFromServoPos(turretServoF.getPosition());
    }

    public Command setTargetServoPosition(double pos) {
        return instant(() -> {
            turretServoF.setPosition(pos);
            turretServoB.setPosition(pos);
        });
    }

    public Command setTurretAngle(double angleRad) {
        return instant(() -> setTargetServoPosition(
                Range.clip(getServoPosFromAngle(angleRad + turretOffsetRad),
                        MIN_SERVO_POS,
                        MAX_SERVO_POS
                )
        ));
    }


    @Override
    public void periodic() {
        if (autoAimEnabled) {
            setTurretAngle(VelocityCompensationCalculator.getTurretAngle());
        }
    }
}
