package org.firstinspires.ftc.teamcode.mechanisms;

import static dev.nextftc.units.Units.Rotations;
import static dev.nextftc.units.Units.RotationsPerMinute;

import org.firstinspires.ftc.teamcode.globals.Constants;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

import org.firstinspires.ftc.teamcode.utils.InterpLUT;
import org.firstinspires.ftc.teamcode.globals.RobotState;

public class Flywheel implements Mechanism {

    private final Drivetrain drivetrain;

    private static final double ENCODER_COUNTS_PER_MOTOR_REV = 28.0;

    private final InterpLUT velocityLUT;
    private final InterpLUT hoodLUT;

    private final NextMotor flywheelMotor1 =
            new NextMotor(
                    "flywheelMotor1",
                    Rotations.of(1.0 / ENCODER_COUNTS_PER_MOTOR_REV)
            );

    private final NextMotor flywheelMotor2 =
            new NextMotor(
                    "flywheelMotor2",
                    Rotations.of(1.0 / ENCODER_COUNTS_PER_MOTOR_REV)
            );

    private final NextServo hoodServo =
            new NextServo("hoodServo");


    private double targetVelocity = 0.0;
    private double targetHoodPosition = 0.0;
    private boolean enabled = false;
    private boolean atSpeed = false;


    public Flywheel(Drivetrain drivetrain) {

        this.drivetrain = drivetrain;

        flywheelMotor2.follow(flywheelMotor1, NextMotor.Direction.REVERSE);

        flywheelMotor1.setZeroPowerBehavior(
                NextMotor.ZeroPowerBehavior.FLOAT
        );

        flywheelMotor2.setZeroPowerBehavior(
                NextMotor.ZeroPowerBehavior.FLOAT
        );


        flywheelMotor1.getVelocityConstants()
                .withP(Constants.Flywheel.kP)
                .withI(Constants.Flywheel.kI)
                .withD(Constants.Flywheel.kD)
                .withS(Constants.Flywheel.kS)
                .withV(Constants.Flywheel.kV)
                .withA(Constants.Flywheel.kA);


        // PLACE HOLDER VALUESSS
        velocityLUT = new InterpLUT()
                .add(40.0, 2000.0)
                .add(60.0, 2300.0)
                .add(80.0, 2600.0)
                .createLUT();

        hoodLUT = new InterpLUT()
                .add(40.0, 0.40)
                .add(60.0, 0.50)
                .add(80.0, 0.60)
                .createLUT();
    }


    @Override
    public void periodic() {

        double distance = getDistanceToGoal();

        targetVelocity = velocityLUT.get(distance);
        targetHoodPosition = hoodLUT.get(distance);

        hoodServo.setPosition(targetHoodPosition);

        if (enabled) {


            flywheelMotor1.setVelocitySetpoint(RotationsPerMinute.of(targetVelocity)
            );


        } else {

            flywheelMotor1.setThrottle(0.0);
            atSpeed = false;
        }


        flywheelMotor1.update();

        if (enabled) {

            double currentVelocity =
                    Math.abs(
                            flywheelMotor1
                                    .getEncoderVelocity().into(RotationsPerMinute)
                    );

            atSpeed =
                    Math.abs(targetVelocity - currentVelocity)
                            < Constants.Flywheel.VELOCITY_TOLERANCE;
        }
    }


    public Command enable() {
        return instant(() -> enabled = true);
    }


    public Command disable() {
        return instant(() -> {
            enabled = false;
            atSpeed = false;
        });
    }


    public boolean isAtSpeed() {
        return atSpeed;
    }


    public double getTargetVelocity() {
        return targetVelocity;
    }

    private double getDistanceToGoal() {

        Pose robotPose = drivetrain.getPose();

        double deltaX =
                RobotState.GOAL_POSE.x() - robotPose.x();

        double deltaY =
                RobotState.GOAL_POSE.y() - robotPose.y();

        return Math.hypot(deltaX, deltaY);
    }

    public double getDistance() {
        return getDistanceToGoal();
    }
}
