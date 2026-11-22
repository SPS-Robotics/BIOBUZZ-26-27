package org.firstinspires.ftc.teamcode.globals;

public final class Constants {

    private Constants() {
    }

    public static class Drivetrain {

        public static double SLOW_MODE_TRIGGER_THRESHOLD = 0.05;
        public static double SLOW_MODE_SCALAR = 0.3;
    }

    public static class Tuning {

        public static double FLYWHEEL_RPM_STEP = 50.0;
        public static double HOOD_SERVO_STEP = 0.01;
    }

    public static class Intake {

        public static double INTAKE_POWER = 1.0;
        public static double OUTTAKE_POWER = -0.6;

        public static double GATE_OPEN = 0.6;
        public static double GATE_CLOSED = 0.17;

        public static long GATE_OPEN_TIME_MS = 400;

        public static double BALL_SENSOR_THRESHOLD_CM = 0.0;
    }
    public static class Flywheel {

        public static double kP = 0.0;
        public static double kI = 0.0;
        public static double kD = 0.0;

        public static double kS = 0.0;
        public static double kV = 0.0;
        public static double kA = 0.0;

        public static double VELOCITY_TOLERANCE = 30.0;

        public static double ENCODER_COUNTS_PER_MOTOR_REV = 28.0;
    }

    public static class Shooter {

        public static double GRAVITY = 386.1; // inches/s^2
        public static double TARGET_ENTRY_ANGLE = 0.0;

        public static double GOAL_HEIGHT = 0.0;
        public static double SHOOTER_EXIT_HEIGHT = 0.0;

        public static double SHOOTER_EXIT_OFFSET = 0.0;

        public static double MIN_LAUNCH_ANGLE =
                Double.NEGATIVE_INFINITY;

        public static double MAX_LAUNCH_ANGLE =
                Double.POSITIVE_INFINITY;


        public static double HOOD_SERVO_1 = 0.0;
        public static double HOOD_ANGLE_1 = 0.0;
        public static double HOOD_SERVO_2 = 0.0;
        public static double HOOD_ANGLE_2 = 0.0;


        public static double FLYWHEEL_RPM_PER_LAUNCH_SPEED = 0.0;
        public static double FLYWHEEL_RPM_INTERCEPT = 0.0;
    }

    public static class Turret {

        public static double kP = 0.0;
        public static double kI = 0.0;
        public static double kD = 0.0;

        public static double RELOC_POS = 0.0;

        public static double TICKS_PER_MOTOR_REV = 28.0;


        public static double TURRET_GEAR_RATIO = 59.42;


        public static double MIN_TICKS = 0.0;
        public static double MAX_TICKS = 0.0;

        public static double CENTRE_OFFSET = 0.0;
        public static double ZERO_ANGLE_OFFSET = Math.PI;


        public static double angularVelocitykV = 0.0;

        public static double POSITION_TOLERANCE_TICKS = 0.0;
    }
}
