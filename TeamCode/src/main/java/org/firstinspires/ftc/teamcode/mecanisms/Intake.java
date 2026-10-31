package org.firstinspires.ftc.teamcode.mecanisms;

import com.pedropathing.ivy.Command; // NextFTC commands extend or use this type
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.sensors.NextDigitalSensor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.measuretypes.Current;

public class Intake implements Mechanism {
    private NextMotor intakeMotorL = new NextMotor("leftIntake");
    private NextMotor intakeMotorR = new NextMotor("rightIntake");
    private NextDigitalSensor beam1 = new NextDigitalSensor("beam1");
    private NextDigitalSensor beam2 = new NextDigitalSensor("beam2");

    public Intake() {
        // Tip: If MotorL follows MotorR, you usually only need to command MotorR!
        intakeMotorL.follow(intakeMotorR, NextMotor.Direction.FORWARD);
        intakeMotorL.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);

        intakeMotorR.setDirection(NextMotor.Direction.REVERSE);
        intakeMotorR.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPower(double power) {
        intakeMotorR.setThrottle(power);
    }

    public Command runIntake(double power) {
        return infinite(() -> setPower(power));
    }

    public boolean isBeamBroken() {
        return beam1.isTriggered() || beam2.isTriggered();
    }

    public Current getCurrent() {
        return intakeMotorL.getCurrent().plus(intakeMotorR.getCurrent());
    }
}
