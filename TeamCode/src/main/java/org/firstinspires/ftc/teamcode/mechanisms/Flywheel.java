package org.firstinspires.ftc.teamcode.mechanisms;

import static dev.nextftc.units.Units.Rotations;
import static dev.nextftc.units.Units.RotationsPerMinute;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.RobotState;
import org.firstinspires.ftc.teamcode.utils.ShooterMath;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

public class Flywheel implements Mechanism {

    private final Drivetrain drivetrain;

    private final NextMotor flywheelMotor1 =
            new NextMotor(
                    "flywheelMotor1",
                    Rotations.of(
                            1.0
                                    / Constants.Flywheel
                                    .ENCODER_COUNTS_PER_MOTOR_REV
                    )
            );

    private final NextMotor flywheelMotor2 =
            new NextMotor(
                    "flywheelMotor2",
                    Rotations.of(
                            1.0
                                    / Constants.Flywheel
                                    .ENCODER_COUNTS_PER_MOTOR_REV
                    )
            );

    private final NextServo hoodServo =
            new NextServo("hoodServo");

    private double targetVelocity = 0.0;
    private double targetHoodPosition = 0.0;

    private boolean enabled = false;
    private boolean atSpeed = false;

    private boolean tuningMode = false;

    private double tuningRPM = 0.0;
    private double tuningHoodPosition = 0.0;

    private boolean tuningHoodCommanded = false;

    private ShooterMath.ShotSolution lastShotSolution =
            ShooterMath.ShotSolution.zero();


    public Flywheel(Drivetrain drivetrain) {

        this.drivetrain = drivetrain;

        flywheelMotor2.follow(
                flywheelMotor1,
                NextMotor.Direction.REVERSE
        );

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
    }


    public Command toggle() {

        return instant(() -> {

            enabled = !enabled;

            if (!enabled) {
                atSpeed = false;
            }
        });
    }


    @Override
    public void periodic() {

        if (tuningMode) {

            targetVelocity =
                    tuningRPM;


            if (tuningHoodCommanded) {

                targetHoodPosition =
                        tuningHoodPosition;

                hoodServo.setPosition(
                        targetHoodPosition
                );
            }


            if (enabled
                    && targetVelocity > 0.0) {

                flywheelMotor1.setVelocitySetpoint(
                        RotationsPerMinute.of(
                                targetVelocity
                        )
                );

            } else {

                flywheelMotor1.setThrottle(0.0);
                atSpeed = false;
            }


            flywheelMotor1.update();


            if (enabled
                    && targetVelocity > 0.0) {

                double currentVelocity =
                        getCurrentVelocity();

                atSpeed =
                        Math.abs(
                                targetVelocity
                                        - currentVelocity
                        )
                                < Constants.Flywheel
                                .VELOCITY_TOLERANCE;

            } else {

                atSpeed = false;
            }


            return;
        }

        lastShotSolution =
                ShooterMath.solve(
                        drivetrain.getPose(),
                        RobotState.GOAL_POSE,
                        drivetrain.getVelocityX(),
                        drivetrain.getVelocityY(),
                        RobotState.SOTM
                );


        if (lastShotSolution.valid) {

            if (ShooterMath.hoodCalibrationReady()) {

                targetHoodPosition =
                        ShooterMath.hoodServoFromLaunchAngle(
                                lastShotSolution.launchAngle
                        );

                hoodServo.setPosition(
                        targetHoodPosition
                );
            }


            if (ShooterMath.flywheelCalibrationReady()) {

                targetVelocity =
                        ShooterMath.flywheelRPMFromLaunchSpeed(
                                lastShotSolution.launchSpeed
                        );

            } else {

                targetVelocity = 0.0;
            }

        } else {

            targetVelocity = 0.0;
            atSpeed = false;
        }


        if (enabled
                && lastShotSolution.valid
                && ShooterMath.flywheelCalibrationReady()) {

            flywheelMotor1.setVelocitySetpoint(
                    RotationsPerMinute.of(
                            targetVelocity
                    )
            );

        } else {

            flywheelMotor1.setThrottle(0.0);
            atSpeed = false;
        }


        flywheelMotor1.update();


        if (enabled
                && lastShotSolution.valid
                && ShooterMath.flywheelCalibrationReady()) {

            double currentVelocity =
                    Math.abs(
                            flywheelMotor1
                                    .getEncoderVelocity()
                                    .into(
                                            RotationsPerMinute
                                    )
                    );

            atSpeed =
                    Math.abs(
                            targetVelocity
                                    - currentVelocity
                    )
                            < Constants.Flywheel
                            .VELOCITY_TOLERANCE;

        } else {

            atSpeed = false;
        }
    }


    public Command enable() {

        return instant(() ->
                enabled = true
        );
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


    public double getTargetHoodPosition() {

        return targetHoodPosition;
    }


    public double getDistance() {

        return lastShotSolution.horizontalDistance;
    }


    public ShooterMath.ShotSolution getShotSolution() {

        return lastShotSolution;
    }

    public double getCurrentVelocity() {
        return Math.abs(
                flywheelMotor1
                        .getEncoderVelocity()
                        .into(RotationsPerMinute)
        );
    }


    public void startTuningMode() {

        tuningMode = true;

        tuningRPM = 0.0;

        tuningHoodPosition =
                hoodServo.getPosition();

        tuningHoodCommanded = false;

        enabled = false;
        atSpeed = false;

        flywheelMotor1.setThrottle(0.0);
        flywheelMotor1.update();
    }


    public void stopTuningMode() {

        tuningMode = false;

        enabled = false;
        atSpeed = false;

        targetVelocity = 0.0;

        flywheelMotor1.setThrottle(0.0);
        flywheelMotor1.update();
    }


    public void adjustTuningRPM(
            double change
    ) {

        tuningRPM =
                Math.max(
                        0.0,
                        tuningRPM + change
                );
    }


    public void adjustTuningHoodPosition(
            double change
    ) {

        tuningHoodPosition =
                Math.max(
                        0.0,
                        Math.min(
                                1.0,
                                tuningHoodPosition + change
                        )
                );

        tuningHoodCommanded = true;
    }


    public void setTuningFlywheelEnabled(
            boolean enabled
    ) {

        this.enabled = enabled;

        if (!enabled) {
            atSpeed = false;
        }
    }


    public double getTuningRPM() {
        return tuningRPM;
    }


    public double getTuningHoodPosition() {
        return tuningHoodPosition;
    }


    public boolean isFlywheelEnabled() {
        return enabled;
    }



}