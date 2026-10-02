package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.globals.Constants;

public final class ShooterMath {

    private static final double EPSILON = 1e-9;

    private ShooterMath() {
    }

    public static class ShotSolution {

        public final boolean valid;

        public final double horizontalDistance;
        public final double heightDifference;
        public final double lineAngle;

        public final double launchAngle;
        public final double launchSpeed;
        public final double timeOfFlight;

        public final double turretOffset;

        public final double radialRobotVelocity;
        public final double tangentialRobotVelocity;

        private ShotSolution(
                boolean valid,
                double horizontalDistance,
                double heightDifference,
                double lineAngle,
                double launchAngle,
                double launchSpeed,
                double timeOfFlight,
                double turretOffset,
                double radialRobotVelocity,
                double tangentialRobotVelocity
        ) {

            this.valid = valid;

            this.horizontalDistance = horizontalDistance;
            this.heightDifference = heightDifference;
            this.lineAngle = lineAngle;

            this.launchAngle = launchAngle;
            this.launchSpeed = launchSpeed;
            this.timeOfFlight = timeOfFlight;

            this.turretOffset = turretOffset;

            this.radialRobotVelocity = radialRobotVelocity;
            this.tangentialRobotVelocity = tangentialRobotVelocity;
        }

        public static ShotSolution zero() {

            return new ShotSolution(
                    false,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0
            );
        }
    }

    public static double launchAngleFromHoodServo(
            double servoPosition
    ) {

        double s1 = Constants.Shooter.HOOD_SERVO_1;
        double a1 = Constants.Shooter.HOOD_ANGLE_1;

        double s2 = Constants.Shooter.HOOD_SERVO_2;
        double a2 = Constants.Shooter.HOOD_ANGLE_2;

        if (Math.abs(s2 - s1) <= EPSILON) {
            return 0.0;
        }

        return ((a2 - a1) / (s2 - s1))
                * (servoPosition - s1)
                + a1;
    }

    public static double hoodServoFromLaunchAngle(
            double launchAngle
    ) {

        double s1 = Constants.Shooter.HOOD_SERVO_1;
        double a1 = Constants.Shooter.HOOD_ANGLE_1;

        double s2 = Constants.Shooter.HOOD_SERVO_2;
        double a2 = Constants.Shooter.HOOD_ANGLE_2;

        if (Math.abs(a2 - a1) <= EPSILON) {
            return 0.0;
        }

        return ((s2 - s1) / (a2 - a1))
                * (launchAngle - a1)
                + s1;
    }

    public static double launchSpeedFromFlywheelRPM(
            double flywheelRPM
    ) {

        double slope =
                Constants.Shooter.FLYWHEEL_RPM_PER_LAUNCH_SPEED;

        if (Math.abs(slope) <= EPSILON) {
            return 0.0;
        }

        return (flywheelRPM
                - Constants.Shooter.FLYWHEEL_RPM_INTERCEPT)
                / slope;
    }

    public static double flywheelRPMFromLaunchSpeed(
            double launchSpeed
    ) {

        double slope =
                Constants.Shooter.FLYWHEEL_RPM_PER_LAUNCH_SPEED;

        if (Math.abs(slope) <= EPSILON) {
            return 0.0;
        }

        return slope * launchSpeed
                + Constants.Shooter.FLYWHEEL_RPM_INTERCEPT;
    }

    public static ShotSolution solveStationary(
            double horizontalDistance,
            double heightDifference
    ) {

        double targetEntryAngle =
                Constants.Shooter.TARGET_ENTRY_ANGLE;

        double gravity =
                Constants.Shooter.GRAVITY;

        if (horizontalDistance <= EPSILON) {
            return ShotSolution.zero();
        }

        double launchAngle =
                Math.atan(
                        (2.0 * heightDifference / horizontalDistance)
                                - Math.tan(targetEntryAngle)
                );

        double denominator =
                2.0
                        * Math.pow(
                        Math.cos(launchAngle),
                        2.0
                )
                        * (
                        horizontalDistance
                                * Math.tan(launchAngle)
                                - heightDifference
                );

        if (denominator <= EPSILON) {
            return ShotSolution.zero();
        }

        double launchSpeedSquared =
                gravity
                        * horizontalDistance
                        * horizontalDistance
                        / denominator;

        if (launchSpeedSquared <= 0.0) {
            return ShotSolution.zero();
        }

        double launchSpeed =
                Math.sqrt(launchSpeedSquared);

        double horizontalLaunchVelocity =
                launchSpeed
                        * Math.cos(launchAngle);

        if (Math.abs(horizontalLaunchVelocity) <= EPSILON) {
            return ShotSolution.zero();
        }

        double timeOfFlight =
                horizontalDistance
                        / horizontalLaunchVelocity;

        return new ShotSolution(
                true,
                horizontalDistance,
                heightDifference,
                0.0,
                launchAngle,
                launchSpeed,
                timeOfFlight,
                0.0,
                0.0,
                0.0
        );
    }

    private static Pose getTurretPivotPose(
            Pose robotPose
    ) {

        return MathUtils.translatePose(
                robotPose,
                -Constants.Turret.CENTRE_OFFSET
        );
    }

    private static Pose getShooterExitPose(
            Pose turretPivotPose,
            double shooterFieldAngle
    ) {

        double exitX =
                turretPivotPose.x()
                        + Math.cos(shooterFieldAngle)
                        * Constants.Shooter.SHOOTER_EXIT_OFFSET;

        double exitY =
                turretPivotPose.y()
                        + Math.sin(shooterFieldAngle)
                        * Constants.Shooter.SHOOTER_EXIT_OFFSET;

        return new Pose(
                exitX,
                exitY,
                shooterFieldAngle
        );
    }

    public static ShotSolution solve(
            Pose robotPose,
            Pose goalPose,
            double robotVelocityX,
            double robotVelocityY,
            boolean velocityCompensation
    ) {

        Pose turretPivotPose =
                getTurretPivotPose(robotPose);

        double pivotDeltaX =
                goalPose.x() - turretPivotPose.x();

        double pivotDeltaY =
                goalPose.y() - turretPivotPose.y();

        double initialLineAngle =
                Math.atan2(
                        pivotDeltaY,
                        pivotDeltaX
                );

        Pose shooterExitPose =
                getShooterExitPose(
                        turretPivotPose,
                        initialLineAngle
                );

        double deltaX =
                goalPose.x() - shooterExitPose.x();

        double deltaY =
                goalPose.y() - shooterExitPose.y();

        double horizontalDistance =
                Math.hypot(
                        deltaX,
                        deltaY
                );

        double lineAngle =
                Math.atan2(
                        deltaY,
                        deltaX
                );

        double heightDifference =
                Constants.Shooter.GOAL_HEIGHT
                        - Constants.Shooter.SHOOTER_EXIT_HEIGHT;

        ShotSolution stationary =
                solveStationary(
                        horizontalDistance,
                        heightDifference
                );

        if (!stationary.valid) {
            return ShotSolution.zero();
        }

        if (!velocityCompensation) {

            return new ShotSolution(
                    true,
                    horizontalDistance,
                    heightDifference,
                    lineAngle,
                    stationary.launchAngle,
                    stationary.launchSpeed,
                    stationary.timeOfFlight,
                    0.0,
                    0.0,
                    0.0
            );
        }

        double robotVelocityMagnitude =
                Math.hypot(
                        robotVelocityX,
                        robotVelocityY
                );

        double robotVelocityAngle =
                Math.atan2(
                        robotVelocityY,
                        robotVelocityX
                );

        double velocityAngleDifference =
                robotVelocityAngle
                        - lineAngle;

        double radialRobotVelocity =
                -Math.cos(
                        velocityAngleDifference
                )
                        * robotVelocityMagnitude;

        double tangentialRobotVelocity =
                Math.sin(
                        velocityAngleDifference
                )
                        * robotVelocityMagnitude;

        double time =
                stationary.timeOfFlight;

        if (time <= EPSILON) {
            return ShotSolution.zero();
        }

        double compensatedRadialVelocity =
                horizontalDistance / time
                        + radialRobotVelocity;

        double newHorizontalVelocity =
                Math.sqrt(
                        compensatedRadialVelocity
                                * compensatedRadialVelocity
                                +
                                tangentialRobotVelocity
                                        * tangentialRobotVelocity
                );

        if (newHorizontalVelocity <= EPSILON) {
            return ShotSolution.zero();
        }

        double verticalVelocity =
                stationary.launchSpeed
                        * Math.sin(
                        stationary.launchAngle
                );

        double compensatedLaunchAngle =
                Math.atan2(
                        verticalVelocity,
                        newHorizontalVelocity
                );

        compensatedLaunchAngle =
                MathUtils.clampValue(
                        compensatedLaunchAngle,
                        Constants.Shooter.MIN_LAUNCH_ANGLE,
                        Constants.Shooter.MAX_LAUNCH_ANGLE
                );

        double compensatedHorizontalDistance =
                newHorizontalVelocity
                        * time;

        double denominator =
                2.0
                        * Math.pow(
                        Math.cos(compensatedLaunchAngle),
                        2.0
                )
                        * (
                        compensatedHorizontalDistance
                                * Math.tan(compensatedLaunchAngle)
                                - heightDifference
                );

        if (denominator <= EPSILON) {
            return ShotSolution.zero();
        }

        double compensatedLaunchSpeedSquared =
                Constants.Shooter.GRAVITY
                        * compensatedHorizontalDistance
                        * compensatedHorizontalDistance
                        / denominator;

        if (compensatedLaunchSpeedSquared <= 0.0) {
            return ShotSolution.zero();
        }

        double compensatedLaunchSpeed =
                Math.sqrt(
                        compensatedLaunchSpeedSquared
                );

        double turretOffset =
                Math.atan2(
                        tangentialRobotVelocity,
                        compensatedRadialVelocity
                );

        return new ShotSolution(
                true,
                horizontalDistance,
                heightDifference,
                lineAngle,
                compensatedLaunchAngle,
                compensatedLaunchSpeed,
                stationary.timeOfFlight,
                turretOffset,
                radialRobotVelocity,
                tangentialRobotVelocity
        );
    }

    public static boolean hoodCalibrationReady() {

        return Math.abs(
                Constants.Shooter.HOOD_ANGLE_2
                        - Constants.Shooter.HOOD_ANGLE_1
        ) > EPSILON
                &&
                Math.abs(
                        Constants.Shooter.HOOD_SERVO_2
                                - Constants.Shooter.HOOD_SERVO_1
                ) > EPSILON;
    }


    public static boolean flywheelCalibrationReady() {

        return Math.abs(
                Constants.Shooter.FLYWHEEL_RPM_PER_LAUNCH_SPEED
        ) > EPSILON;
    }
}