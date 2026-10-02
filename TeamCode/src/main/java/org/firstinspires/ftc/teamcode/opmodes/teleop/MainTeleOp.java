package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.commands.Commands;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.globals.RobotState;
import org.firstinspires.ftc.teamcode.utils.ShooterMath;

import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;


@NextTeleop(name = "BIOBUZZ Teleop")
public class MainTeleOp extends NextOpMode {

    private final Robot robot;

    public MainTeleOp(Robot robot) {
        super(robot);
        this.robot = robot;

        robot.drivetrain.initialize(hardwareMap);
        robot.turret.initialize(hardwareMap);
    }

    @Override
    public void start(){
        RobotState.SOTM = false;
        CommandGamepad driver = new CommandGamepad(gamepad1);

        robot.drivetrain.startDrive(gamepad1);

        driver.rightTrigger().isOver(0.1)
                .onTrue(robot.intake.run())
                .onFalse(robot.intake.stop());

        driver.cross()
                .onTrue(robot.intake.outtake())
                .onFalse(robot.intake.stop());

        driver.rightBumper().onTrue(
                sequential(
                        robot.intake.openGate(),
                        robot.intake.run()
                )
        );

        driver.rightBumper().onFalse(
                parallel(
                        robot.intake.stop(),
                        robot.intake.closeGate()
                )
        );

        driver.triangle()
                .onTrue(robot.turret.toggleTracking());

        driver.circle()
                .onTrue(robot.flywheel.toggle());

        // reset heading for feild centric view
        driver.start().onTrue(robot.drivetrain.resetHeading());

        driver.square().onTrue(
                Commands.instant(() ->
                        RobotState.SOTM = !RobotState.SOTM
                )
        );


    }

    @Override
    public void periodic() {

        ShooterMath.ShotSolution shot =
                robot.flywheel.getShotSolution();

        telemetry.addData(
                "SOTM",
                RobotState.SOTM
        );

        telemetry.addData(
                "Shot Valid",
                shot.valid
        );

        telemetry.addData(
                "Ready To Shoot",
                robot.readyToShoot()
        );

        telemetry.addData(
                "Distance",
                shot.horizontalDistance
        );

        telemetry.addData(
                "Launch Angle",
                Math.toDegrees(
                        shot.launchAngle
                )
        );

        telemetry.addData(
                "Launch speed",
                shot.launchSpeed
        );

        telemetry.addData(
                "tof",
                shot.timeOfFlight
        );

        telemetry.addData(
                "Target RPM",
                robot.flywheel.getTargetVelocity()
        );

        telemetry.addData(
                "Current RPM",
                robot.flywheel.getCurrentVelocity()
        );

        telemetry.addData(
                "Hood Position",
                robot.flywheel.getTargetHoodPosition()
        );

        telemetry.addData(
                "Turret Target",
                robot.turret.getTargetPosition()
        );

        telemetry.addData(
                "Turret pos",
                robot.turret.getTurretPosition()
        );

        telemetry.addData(
                "TurretOffset Degrees",
                Math.toDegrees(
                        shot.turretOffset
                )
        );

        telemetry.addData(
                "Radial Velocity",
                shot.radialRobotVelocity
        );

        telemetry.addData(
                "Tangential Velocity",
                shot.tangentialRobotVelocity
        );

        telemetry.update();
    }

}
