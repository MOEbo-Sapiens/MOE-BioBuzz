package org.firstinspires.ftc.teamcode.pedropathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.helpers.Config;

public class Constants {
    public static Follower create(HardwareMap h) {
        // return new Follower(Drivetrain, Localizer, Foresight);
        return null;
    }

    public static MecanumConfig driveConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set(Config.frontLeftName);
                c.backLeftName.set(Config.backLeftName);
                c.frontRightName.set(Config.frontRightName);
                c.backRightName.set(Config.backRightName);

                c.frontLeftDirection.set(Config.frontLeftDirection.getSdkDirection());
                c.backLeftDirection.set(Config.backLeftDirection.getSdkDirection());
                c.frontRightDirection.set(Config.frontRightDirection.getSdkDirection());
                c.backRightDirection.set(Config.backRightDirection.getSdkDirection());
            }
    );
}