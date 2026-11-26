
package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.globals.RobotState;
import org.firstinspires.ftc.teamcode.utils.ShooterMath;

import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

@NextTeleop(name = "BIOBUZZ Teleop")
public class MainTeleOp extends NextOpMode {

    private final Robot robot;
    private boolean previousDpadRight = false;
    private boolean started = false;

    private boolean previousDpadLeft = false;

    public MainTeleOp(Robot robot) {
        super(robot);
        this.robot = robot;

        robot.drivetrain.initialize(hardwareMap);
        robot.turret.initialize(hardwareMap);
        robot.intake.initialize(hardwareMap);
    }

    @Override
    public void start() {
        RobotState.SOTM = false;
        RobotState.selectAlliance(RobotState.alliance);

        previousDpadRight = gamepad1.dpad_right;

        previousDpadLeft = gamepad1.dpad_left;

        CommandGamepad driver = new CommandGamepad(gamepad1);

        robot.drivetrain.startDrive(gamepad1);
        robot.intake.setManualControls(false, false, false);

        driver.triangle()
                .onTrue(robot.turret.toggleTracking());

        driver.circle()
                .onTrue(robot.flywheel.toggle());

        driver.start()
                .onTrue(robot.drivetrain.resetHeading());

        started = true;
    }

    @Override
    public void periodic() {

        if (!started) {
            return;
        }

        robot.intake.setManualControls(
                gamepad1.right_trigger > 0.1,
                gamepad1.cross,
                gamepad1.right_bumper
        );

        if (gamepad1.dpad_up) {
            RobotState.selectAlliance(
                    RobotState.Alliance.RED
            );
        } else if (gamepad1.dpad_down) {
            RobotState.selectAlliance(
                    RobotState.Alliance.BLUE
            );
        }


        if (gamepad1.dpad_left && !previousDpadLeft) {
            robot.turret.setManualHome();
        }

        previousDpadLeft = gamepad1.dpad_left;


        if (gamepad1.dpad_right && !previousDpadRight) {
            RobotState.toggleCell();
        }

        previousDpadRight = gamepad1.dpad_right;

        telemetry.addData("Alliance", RobotState.alliance);
        telemetry.addData("HIVE Cell", RobotState.activeCell + 1);
        telemetry.addData("Target Configured", RobotState.hasTarget());
        telemetry.addData("Turret Homed", robot.turret.isHomed());
        telemetry.addData(
                "Turret Tracking",
                robot.turret.isTracking()
        );

        telemetry.addData("Turret Position", robot.turret.getTurretPosition());
        telemetry.addData("Current RPM", robot.flywheel.getCurrentVelocity());
        telemetry.addData("Target RPM", robot.flywheel.getTargetVelocity());

        if (RobotState.hasTarget()) {
            ShooterMath.ShotSolution shot =
                    robot.flywheel.getShotSolution();

            telemetry.addData("Shot Valid", shot.valid);
            telemetry.addData("Ready To Shoot", robot.readyToShoot());
            telemetry.addData("Distance", shot.horizontalDistance);
            telemetry.addData("Launch Angle",
                    Math.toDegrees(shot.launchAngle));
            telemetry.addData("Launch Speed", shot.launchSpeed);
            telemetry.addData("Hood Position",
                    robot.flywheel.getTargetHoodPosition());
            telemetry.addData("Turret Target",
                    robot.turret.getTargetPosition());
        }

        telemetry.addData("Intake",
                gamepad1.right_trigger > 0.1);
        telemetry.addData("Feeding", gamepad1.right_bumper);
        telemetry.addData("Reverse", gamepad1.cross);

        telemetry.update();
    }
}
