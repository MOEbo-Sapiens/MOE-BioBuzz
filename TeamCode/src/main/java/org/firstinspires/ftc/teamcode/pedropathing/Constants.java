package org.firstinspires.ftc.teamcode.pedropathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.helpers.RobotConfig;

public class Constants {
    public static Follower create(HardwareMap h) {
        // return new Follower(Drivetrain, Localizer, Foresight);
        return null;
    }

    public static MecanumConfig driveConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set(RobotConfig.frontLeftName);
                c.backLeftName.set(RobotConfig.backLeftName);
                c.frontRightName.set(RobotConfig.frontRightName);
                c.backRightName.set(RobotConfig.backRightName);

                c.frontLeftDirection.set(RobotConfig.frontLeftDirection.getSdkDirection());
                c.backLeftDirection.set(RobotConfig.backLeftDirection.getSdkDirection());
                c.frontRightDirection.set(RobotConfig.frontRightDirection.getSdkDirection());
                c.backRightDirection.set(RobotConfig.backRightDirection.getSdkDirection());
            }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(6.2501141405481055);
        c.yPodOffset.set(-3.917391018604669);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });


}