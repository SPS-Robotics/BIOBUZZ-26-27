package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.teamcode.Robot;

import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;

@NextTeleop(
        name = "Shooter Tuner",
        group = "Tuning"
)
public class ShooterTuner extends NextOpMode {

    private final Robot robot;

    private boolean previousDpadUp = false;
    private boolean previousDpadDown = false;

    private boolean previousDpadLeft = false;
    private boolean previousDpadRight = false;

    private boolean previousCircle = false;


    public ShooterTuner(
            Robot robot
    ) {

        super(robot);

        this.robot = robot;

        robot.drivetrain.initialize(
                hardwareMap
        );

        robot.turret.initialize(
                hardwareMap
        );
    }


    @Override
    public void start() {

        robot.flywheel.startTuningMode();
    }


    @Override
    public void periodic() {

        if (gamepad1.dpad_up
                && !previousDpadUp) {

            robot.flywheel.adjustTuningRPM(
                    50.0
            );
        }


        if (gamepad1.dpad_down
                && !previousDpadDown) {

            robot.flywheel.adjustTuningRPM(
                    -50.0
            );
        }


        if (gamepad1.dpad_right
                && !previousDpadRight) {

            robot.flywheel
                    .adjustTuningHoodPosition(
                            0.01
                    );
        }


        if (gamepad1.dpad_left
                && !previousDpadLeft) {

            robot.flywheel
                    .adjustTuningHoodPosition(
                            -0.01
                    );
        }


        if (gamepad1.circle
                && !previousCircle) {

            robot.flywheel
                    .setTuningFlywheelEnabled(
                            !robot.flywheel
                                    .isFlywheelEnabled()
                    );
        }


        previousDpadUp =
                gamepad1.dpad_up;

        previousDpadDown =
                gamepad1.dpad_down;

        previousDpadLeft =
                gamepad1.dpad_left;

        previousDpadRight =
                gamepad1.dpad_right;

        previousCircle =
                gamepad1.circle;


        telemetry.addData(
                "Flywheel Running",
                robot.flywheel
                        .isFlywheelEnabled()
        );

        telemetry.addData(
                "Target RPM",
                robot.flywheel
                        .getTuningRPM()
        );

        telemetry.addData(
                "Current RPM",
                robot.flywheel
                        .getCurrentVelocity()
        );

        telemetry.addData(
                "Hood Servo Position",
                robot.flywheel
                        .getTuningHoodPosition()
        );

        telemetry.addData(
                "At Speed",
                robot.flywheel
                        .isAtSpeed()
        );

        telemetry.addLine("");

        telemetry.addLine(
                "D-Pad Up/Down: RPM +/- 50"
        );

        telemetry.addLine(
                "D-Pad Left/Right: Hood +/- 0.01"
        );

        telemetry.addLine(
                "Circle: Flywheel on/off"
        );

        telemetry.update();
    }


    @Override
    public void end() {

        robot.flywheel.stopTuningMode();
    }
}