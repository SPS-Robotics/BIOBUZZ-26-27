package org.firstinspires.ftc.teamcode.mechanisms;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

public class Intake implements Mechanism {

    private static final double INTAKE_POWER = 1.0;
    private static final double OUTTAKE_POWER = -0.6;

    private static final double GATE_OPEN = 0.6;
    private static final double GATE_CLOSED = 0.17;

    private static final long GATE_OPEN_TIME_MS = 400;

    private final NextMotor motor = new NextMotor("intakeMotor");
    private final NextServo gateServo = new NextServo("gateServo");


    public Command run() {
        return instant(() -> motor.setThrottle(INTAKE_POWER));
    }

    public Command outtake() {
        return instant(() -> motor.setThrottle(OUTTAKE_POWER));
    }

    public Command stop() {
        return instant(() -> motor.setThrottle(0.0));
    }

    public Command openGate() {
        return sequential(
                instant(() -> gateServo.setPosition(GATE_OPEN)),
                waitMs(GATE_OPEN_TIME_MS)
        );}

    public Command closeGate() {
        return instant(() -> gateServo.setPosition(GATE_CLOSED));
    }



}
