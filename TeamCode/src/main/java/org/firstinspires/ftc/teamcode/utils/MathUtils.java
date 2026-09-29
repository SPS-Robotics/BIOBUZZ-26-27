package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.globals.Constants;

public class MathUtils {

    public static double calculateAngleToPose(
            Pose turretPose,
            Pose targetPose
    ) {

        double deltaX =
                targetPose.x() - turretPose.x();

        double deltaY =
                targetPose.y() - turretPose.y();

        double angleToTarget =
                Math.atan2(deltaY, deltaX);

        double rawDelta =
                angleToTarget
                        - (turretPose.heading()
                                + Constants.Turret.ZERO_ANGLE_OFFSET
                );

        return Math.atan2(
                Math.sin(rawDelta),
                Math.cos(rawDelta)
        );
    }


    public static Pose translatePose(
            Pose original,
            double distance
    ) {

        double translatedX =
                original.x()
                        + Math.cos(original.heading())
                        * distance;

        double translatedY =
                original.y()
                        + Math.sin(original.heading())
                        * distance;

        return new Pose(
                translatedX,
                translatedY,
                original.heading()
        );
    }


    public static double clampValue(
            double value,
            double minimum,
            double maximum
    ) {

        return Math.max(
                minimum,
                Math.min(value, maximum)
        );
    }
}