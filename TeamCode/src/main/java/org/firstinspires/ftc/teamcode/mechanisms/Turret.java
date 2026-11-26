package org.firstinspires.ftc.teamcode.mechanisms;

import static dev.nextftc.units.Units.Radians;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.RobotState;
import org.firstinspires.ftc.teamcode.utils.MathUtils;
import org.firstinspires.ftc.teamcode.utils.ShooterMath;

import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedback.PIDController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Turret implements Mechanism {

    private final Drivetrain drivetrain;

    private TouchSensor magneticLimitSwitch;

    private double encoderOffset = 0.0;

    private boolean wasLimitSwitchPressed = false;

    private boolean homed = false;

    private boolean turretTracking = false;

    private boolean tuningMode = false;
    private double tuningPower = 0.0;

    public Turret(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        turretRotator.setZeroPowerBehavior(
                NextMotor.ZeroPowerBehavior.BRAKE
        );
    }

    private final NextMotor turretRotator =
            new NextMotor("turretRotator");

    private DcMotorEx turretEncoder;

    private final PIDController controller =
            new PIDController(
                    new PIDCoefficients(
                            Constants.Turret.kP,
                            Constants.Turret.kI,
                            Constants.Turret.kD
                    )
            );


    public void initialize(HardwareMap hardwareMap) {

        magneticLimitSwitch = hardwareMap.tryGet(
                TouchSensor.class,
                "magneticLimitSwitch"
        );

        turretEncoder = hardwareMap.get(
                DcMotorEx.class,
                "turretRotator"
        );
    }


    public Command toggleTracking() {
        return instant(() -> {
            if (turretTracking) {
                turretTracking = false;
            } else {
                turretTracking =
                        homed
                                && Constants.Turret.MIN_TICKS
                                < Constants.Turret.MAX_TICKS
                                && Constants.Turret.kP > 0.0;
            }
        });
    }


    public double getRawTurretPosition() {
        return turretEncoder.getCurrentPosition();
    }

    public boolean isLimitSwitchPressed() {
        return magneticLimitSwitch != null
                && magneticLimitSwitch.isPressed();
    }


    public double getTurretPosition() {

        return getRawTurretPosition()
                + encoderOffset;
    }

    public boolean isHomed() {
        return homed;
    }


    public void setManualHome() {
        turretTracking = false;
        turretRotator.setThrottle(0.0);

        encoderOffset =
                Constants.Turret.RELOC_POS
                        - getRawTurretPosition();

        homed = true;
    }

    public boolean isTracking() {
        return turretTracking;
    }


    private void updateHoming() {

        boolean switchPressed =
                magneticLimitSwitch != null
                        && magneticLimitSwitch.isPressed();


        if (switchPressed && !wasLimitSwitchPressed) {

            encoderOffset =
                    Constants.Turret.RELOC_POS
                            - getRawTurretPosition();

            homed = true;
        }



        wasLimitSwitchPressed = switchPressed;
    }
    public double calculateTurretPosition(Pose goalPose) {

        Pose robotPose = drivetrain.getPose();

        Pose turretPose =
                MathUtils.translatePose(
                        robotPose,
                        -Constants.Turret.CENTRE_OFFSET
                );

        double turretAngleRadians =
                MathUtils.calculateAngleToPose(
                        turretPose,
                        goalPose
                );

        if (RobotState.SOTM) {

            ShooterMath.ShotSolution shot =
                    ShooterMath.solve(
                            robotPose,
                            goalPose,
                            drivetrain.getVelocityX(),
                            drivetrain.getVelocityY(),
                            true
                    );

            if (shot.valid) {
                turretAngleRadians -= shot.turretOffset;
            }
        }

        double targetTicks =
                (turretAngleRadians / (2.0 * Math.PI))
                        * Constants.Turret.TICKS_PER_MOTOR_REV
                        * Constants.Turret.TURRET_GEAR_RATIO;

        return MathUtils.clampValue(
                targetTicks,
                Constants.Turret.MIN_TICKS,
                Constants.Turret.MAX_TICKS
        );
    }

    public double getTargetPosition() {
        return calculateTurretPosition(
                RobotState.GOAL_POSE
        );
    }


    public Command enableTracking() {
        return instant(() -> {
            turretTracking =
                    homed
                            && Constants.Turret.MIN_TICKS
                            < Constants.Turret.MAX_TICKS
                            && Constants.Turret.kP > 0.0;
        });
    }


    public Command disableTracking() {
        return instant(() -> turretTracking = false);
    }

    public void startTuningMode() {

        tuningMode = true;
        turretTracking = false;
        tuningPower = 0.0;
    }

    public void stopTuningMode() {

        tuningPower = 0.0;
        tuningMode = false;

        turretRotator.setThrottle(0.0);
    }

    public void setTuningPower(double power) {

        tuningPower =
                MathUtils.clampValue(
                        power,
                        -1.0,
                        1.0
                );
    }

    @Override
    public void periodic() {

        updateHoming();

        if (tuningMode) {

            turretRotator.setThrottle(
                    tuningPower
            );

            return;
        }

        double targetPosition =
                calculateTurretPosition(
                        RobotState.GOAL_POSE
                );

        double currentPosition =
                getTurretPosition();

        double error =
                targetPosition - currentPosition;

        double power =
                controller.calculate(error);

        power +=
                -drivetrain.getAngularVelocity()
                        * Constants.Turret.angularVelocitykV;

        power = MathUtils.clampValue(
                power,
                -1.0,
                1.0
        );



        if (currentPosition <= Constants.Turret.MIN_TICKS
                && power < 0.0) {

            power = 0.0;
        }

        if (currentPosition >= Constants.Turret.MAX_TICKS
                && power > 0.0) {

            power = 0.0;
        }


        if (!turretTracking || !homed) {
            power = 0.0;
        }


        turretRotator.setThrottle(power);
    }

    public boolean isAtTarget() {

        return Math.abs(
                getTargetPosition()
                        - getTurretPosition()
        ) < Constants.Turret.POSITION_TOLERANCE_TICKS;
    }

    public double getCurrentPosition() {
        return getTurretPosition();
    }
}