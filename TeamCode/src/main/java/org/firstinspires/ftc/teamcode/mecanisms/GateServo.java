package org.firstinspires.ftc.teamcode.mecanisms;



import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.hardware.lynx.NextLynxModule;
import dev.nextftc.robot.Mechanism;

public class GateServo implements Mechanism {
    private final NextLynxModule gateController = RobotController.controlHub();
    private final int gatePort = 0;

    NextServo gateServo = new NextServo(gateController, gatePort);

    public static double openGatePosition = 0.4;
    public static double closedGatePosition = 0.48;

    public Command open() { return instant(() -> gateServo.setPosition(openGatePosition)); }
    public Command close() { return instant(() -> gateServo.setPosition(closedGatePosition)); }

}
