package org.firstinspires.ftc.teamcode.mecanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.helpers.RobotConfig;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Measure;
import dev.nextftc.units.Units;
import dev.nextftc.units.measuretypes.AngularVelocity;
import dev.nextftc.units.measuretypes.Current;
import dev.nextftc.units.unittypes.AngularVelocityUnit;

@Config
public class Flywheel implements Mechanism {
    private NextMotor shooterMotorL = new NextMotor("leftFlywheel");
    private NextMotor shooterMotorR = new NextMotor("rightFlywheel");

    public static double kV = 0.000375, kP = 0.003; //TODO: TUNE VALUES
    public static AngularVelocity kS = new AngularVelocity(0.08, Units.RadiansPerSecond);

    private AngularVelocity target = new AngularVelocity(0, Units.RadiansPerSecond);
    private boolean activated = false;

    VoltageSensor voltageSensor;

    public Flywheel(HardwareMap hardwareMap, VoltageSensor voltageSensor) {
        this.voltageSensor = voltageSensor;

        shooterMotorL.setDirection(NextMotor.Direction.FORWARD);
        shooterMotorR.setDirection(NextMotor.Direction.REVERSE);

        shooterMotorL.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.FLOAT);
        shooterMotorR.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.FLOAT);
    }

    /**
     *
     * @param ticksPerSecond motor speed in ticks per second
     * @return flywheel speed in radians per second
     */
    public static double motorTicksToFlywheelRadians(double ticksPerSecond) {
        //28 -- ticks per rotation
        // 1.4 -- ratio of flywheel velocity to motor velocity
        return (ticksPerSecond / 28) * 2 * Math.PI * 1.4;
    }

    /**
     *
     * @param radiansPerSecond flywheel speed in radians per second
     * @return motor speed in ticks per second
     */
    public static double flywheelRadiansToMotorTicks(double radiansPerSecond) {
        //28 -- ticks per rotation
        // 1.4 -- ratio of flywheel velocity to motor velocity
        return (radiansPerSecond / (2 * Math.PI * 1.4)) * 28;
    }


    public AngularVelocity getCurrentAngularVel() {
        return shooterMotorL.getEncoderVelocity();
    }

    public void setTargetAngularVelocity(AngularVelocity target) {
        this.target = target;
    }


    public AngularVelocity getTargetAngularVelocity() {
        return target;
    }

    public void setPower(AngularVelocity power) {
        shooterMotorL.setVelocitySetpoint(power);
        shooterMotorR.setVelocitySetpoint(power);
    }

    public void deactivate() {
        activated = false;
        setPower(new AngularVelocity(0, Units.RadiansPerSecond));
    }

    public void activate() {
        activated = true;
    }

    public boolean isReady() {
        return getTargetAngularVelocity().isNear(getCurrentAngularVel(), 40);
    }

    public boolean getActivated() {
        return activated;
    }

    public Current getCurrent() {
        return shooterMotorL.getCurrent().plus(shooterMotorR.getCurrent());
    }

    public void toggle() {
        activated = !activated;
        if (!activated) {
            setPower(new AngularVelocity(0, Units.RadiansPerSecond));
        }
    }

    public void update() {
        if (activated) {
            AngularVelocity power =  (getTargetAngularVelocity().times(kV)).plus(getTargetAngularVelocity().minus(getCurrentAngularVel()).times(kP).plus(kS));
            power.times(12)
                    .div(voltageSensor.getVoltage());
            setPower(power);
        }
    }
}