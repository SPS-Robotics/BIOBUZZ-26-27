package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.teamcode.Robot;

import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;

@NextTeleop(
        name = "Turret Tuner",
        group = "Tuning"
)
public class TurretTuner extends NextOpMode {

    private final Robot robot;

    private static final double TUNING_POWER = 0.15;


    public TurretTuner(Robot robot) {

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

        robot.turret.startTuningMode();
    }


    @Override
    public void periodic() {

        double power = 0.0;


        if (gamepad1.dpad_left) {

            power = -TUNING_POWER;

        } else if (gamepad1.dpad_right) {

            power = TUNING_POWER;
        }


        robot.turret.setTuningPower(
                power
        );


        telemetry.addData(
                "Raw encoder value",
                robot.turret.getRawTurretPosition()
        );

        telemetry.addData(
                "Turret Position",
                robot.turret.getTurretPosition()
        );

        telemetry.addData(
                "Limit Switch",
                robot.turret.isLimitSwitchPressed()
        );

        telemetry.addData(
                "homed",
                robot.turret.isHomed()
        );

        telemetry.addData(
                "Manual Power",
                power
        );


        telemetry.addLine("");

        telemetry.addLine(
                "dpad left or right: move turret"
        );

        telemetry.update();
    }


    @Override
    public void end() {

        robot.turret.stopTuningMode();
    }
}