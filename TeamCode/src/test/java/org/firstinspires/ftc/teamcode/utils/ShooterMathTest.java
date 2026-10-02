package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.globals.Constants;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class ShooterMathTest {

    @Test
    public void testShooterMath() {

        Constants.Shooter.TARGET_ENTRY_ANGLE =
                Math.toRadians(-45.0);

        Constants.Shooter.GOAL_HEIGHT =
                48.0;

        Constants.Shooter.SHOOTER_EXIT_HEIGHT =
                24.0;

        Constants.Shooter.SHOOTER_EXIT_OFFSET =
                0.0;

        Constants.Shooter.MIN_LAUNCH_ANGLE =
                Math.toRadians(20.0);

        Constants.Shooter.MAX_LAUNCH_ANGLE =
                Math.toRadians(75.0);

        Constants.Turret.CENTRE_OFFSET =
                0.0;


        Pose robotPose =
                new Pose(
                        0.0,
                        0.0,
                        0.0
                );

        Pose goalPose =
                new Pose(
                        100.0,
                        0.0,
                        0.0
                );


        ShooterMath.ShotSolution stationary =
                ShooterMath.solve(
                        robotPose,
                        goalPose,
                        0.0,
                        0.0,
                        false
                );


        ShooterMath.ShotSolution zeroVelocitySOTM =
                ShooterMath.solve(
                        robotPose,
                        goalPose,
                        0.0,
                        0.0,
                        true
                );


        ShooterMath.ShotSolution movingTowardGoal =
                ShooterMath.solve(
                        robotPose,
                        goalPose,
                        20.0,
                        0.0,
                        true
                );


        ShooterMath.ShotSolution movingAwayFromGoal =
                ShooterMath.solve(
                        robotPose,
                        goalPose,
                        -20.0,
                        0.0,
                        true
                );


        ShooterMath.ShotSolution movingSideways =
                ShooterMath.solve(
                        robotPose,
                        goalPose,
                        0.0,
                        20.0,
                        true
                );


        System.out.println(
                "stationaty valid: "
                        + stationary.valid
        );

        System.out.println(
                "stationary speed: "
                        + stationary.launchSpeed
        );

        System.out.println(
                "stationary angle: "
                        + Math.toDegrees(
                        stationary.launchAngle
                )
        );

        System.out.println(
                "stationary tof: "
                        + stationary.timeOfFlight
        );


        System.out.println(
                "Zero velocity SOTM speed: "
                        + zeroVelocitySOTM.launchSpeed
        );


        System.out.println(
                "to goal speed: "
                        + movingTowardGoal.launchSpeed
        );


        System.out.println(
                "away from goal speed: "
                        + movingAwayFromGoal.launchSpeed
        );


        System.out.println(
                "lateral speed: "
                        + movingSideways.launchSpeed
        );

        System.out.println(
                "sideways turret offset deg: "
                        + Math.toDegrees(
                        movingSideways.turretOffset
                )
        );

        assertTrue(stationary.valid);

        assertTrue(zeroVelocitySOTM.valid);

        assertEquals(
                stationary.launchSpeed,
                zeroVelocitySOTM.launchSpeed,
                0.001
        );

        assertTrue(
                movingTowardGoal.launchSpeed
                        < stationary.launchSpeed
        );

        assertTrue(
                movingAwayFromGoal.launchSpeed
                        > stationary.launchSpeed
        );

        assertTrue(
                Math.abs(movingSideways.turretOffset)
                        > 0.001
        );
    }
}