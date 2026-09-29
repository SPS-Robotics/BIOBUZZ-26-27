package org.firstinspires.ftc.teamcode.globals;

public final class Constants {

    private Constants() {
    }

    public static class Flywheel {

        public static double kP = 0.0;
        public static double kI = 0.0;
        public static double kD = 0.0;

        public static double kS = 0.0;
        public static double kV = 0.0;
        public static double kA = 0.0;

        public static double VELOCITY_TOLERANCE = 30.0;
    }

    public static class Turret {

        public static double kP = 0.0;
        public static double kI = 0.0;
        public static double kD = 0.0;

        public static double RELOC_POS = 0.0;

        public static double TICKS_PER_MOTOR_REV = 0.0;


        public static double TURRET_GEAR_RATIO = 0.0;


        public static double MIN_TICKS = 0.0;
        public static double MAX_TICKS = 0.0;

        public static double CENTRE_OFFSET = 0.0;
        public static double ZERO_ANGLE_OFFSET = Math.PI;


        public static double angularVelocitykV = 0.0;
    }
}