package org.firstinspires.ftc.teamcode.pedropathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.helpers.RobotConfig;

public class Constants {
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, driveConfig),
                new Foresight(foresightConfig)
        );
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

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3106847485432158);
                Controller secondaryTranslationalForward = Controller.proportional(0.11478971293734351);
                Controller primaryTranslationalLateral = Controller.proportional(0.6293020075277524);
                Controller secondaryTranslationalLateral = Controller.proportional(0.23251027652217232);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.012083748985364432));
                c.brake.set(Controller.proportionalFeedforward(0.010271186637559767));

                c.headingFeedback.set(Controller.proportional(5.752843168584152));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06512130157293786, 0.00610357294959225));

                c.linearBrakeCoefficients.set(Matrix.diag(0.18194572794123345, 0.08387252993401775));
                c.quadraticBrakeCoefficients.set(Matrix.diag(9.139007319973884E-4, 0.0021721408455285694));

                c.maxAchievableForwardVelocity.set(85.23531648040543);
                c.maxAchievableStrafeVelocity.set(65.83782103864594);
                c.naturalForwardDeceleration.set(33.20152811273069);
                c.naturalStrafeDeceleration.set(71.83728977919587);
            }
    );


}