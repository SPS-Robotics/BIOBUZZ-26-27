
package org.firstinspires.ftc.teamcode.globals;

import com.pedropathing.math.Pose;

public class RobotState {

    public enum Alliance {
        BLUE, RED;
    }

    public static Alliance alliance = Alliance.BLUE;

    public static int activeCell = 0;


    public static Pose BLUE_CELL_0 = new Pose(83.50, 87.24);
    public static Pose BLUE_CELL_1 = new Pose(83.50, 54.26);


    public static double FIELD_CENTRE_X = 70.75;
    public static double FIELD_CENTRE_Y = 70.75;

    public static Pose GOAL_POSE =
            BLUE_CELL_0;

    public static boolean SOTM = false;

    private static Pose selectedBlueCell() {
        return activeCell == 0
                ? BLUE_CELL_0
                : BLUE_CELL_1;
    }

    public static boolean hasTarget() {
        return selectedBlueCell() != null;
    }

    public static void updateGoalPose() {

        Pose blue = selectedBlueCell();

        if (blue == null) {
            return;
        }

        if (alliance == Alliance.BLUE) {
            GOAL_POSE = blue;
        } else {
            GOAL_POSE = new Pose(
                    2.0 * FIELD_CENTRE_X - blue.x(),
                    2.0 * FIELD_CENTRE_Y - blue.y()
            );
        }
    }

    public static void selectAlliance(Alliance selected) {
        alliance = selected;
        updateGoalPose();
    }

    public static void toggleCell() {
        activeCell = 1 - activeCell;
        updateGoalPose();
    }
}
