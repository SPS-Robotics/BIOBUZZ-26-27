package org.firstinspires.ftc.teamcode.mechanisms;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.globals.Constants;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake implements Mechanism {

    private final NextMotor motor = new NextMotor("intakeMotor");
    private final NextServo gateServo = new NextServo("gateServo");

    private RevColorSensorV3 leftBallSensor;
    private RevColorSensorV3 rightBallSensor;

    private boolean leftBallPresent = false;
    private boolean rightBallPresent = false;

    public boolean isLeftBallPresent() {
        return leftBallPresent;
    }

    public boolean isRightBallPresent() {
        return rightBallPresent;
    }

    public boolean isFull() {
        return leftBallPresent
                && rightBallPresent;
    }

    public double getLeftBallDistanceCm() {

        if (leftBallSensor == null) {
            return Double.NaN;
        }

        return leftBallSensor.getDistance(
                DistanceUnit.CM
        );
    }

    public double getRightBallDistanceCm() {

        if (rightBallSensor == null) {
            return Double.NaN;
        }

        return rightBallSensor.getDistance(
                DistanceUnit.CM
        );
    }


    public void initialize(HardwareMap hardwareMap) {

        leftBallSensor =
                hardwareMap.get(
                        RevColorSensorV3.class,
                        "leftBallSensor"
                );

        rightBallSensor =
                hardwareMap.get(
                        RevColorSensorV3.class,
                        "rightBallSensor"
                );
    }

    private boolean isBallPresent(RevColorSensorV3 sensor)
    {

        double distance =
                sensor.getDistance(
                        DistanceUnit.CM
                );

        return !Double.isNaN(distance)
                && distance
                <= Constants.Intake.BALL_SENSOR_THRESHOLD_CM;
    }


    public Command run() {
        return instant(() -> motor.setThrottle(Constants.Intake.INTAKE_POWER));
    }

    public Command outtake() {
        return instant(() -> motor.setThrottle(Constants.Intake.OUTTAKE_POWER));
    }

    public Command stop() {
        return instant(() -> motor.setThrottle(0.0));
    }

    public Command openGate() {
        return sequential(
                instant(() -> gateServo.setPosition(Constants.Intake.GATE_OPEN)),
                waitMs(Constants.Intake.GATE_OPEN_TIME_MS)
        );}

    public Command closeGate() {
        return instant(() -> gateServo.setPosition(Constants.Intake.GATE_CLOSED));
    }

    @Override
    public void periodic() {

        if (leftBallSensor == null
                || rightBallSensor == null) {
            return;
        }

        leftBallPresent =
                isBallPresent(leftBallSensor);

        rightBallPresent =
                isBallPresent(rightBallSensor);
    }



}
