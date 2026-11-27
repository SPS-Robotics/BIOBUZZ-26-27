/*
  Things to Fix
 Constants Problems in globals constants
 -    flywheel
  -    hood
  -    turret
  -    GOAL_POSE
  - maybe more i'm not entirely sure
  - Timing (30s only) have to check if the timing actually works, like pretty much everything else this just needs testing on a correct game field.
  - Red side only.
  - Correct size Bot? adjust based on size of bot, rn its 16"
  - Tune SPIN_UP_TIMEOUT_MS / FEED_TIME_MS on the real field
 */
package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.globals.RobotState;
import org.firstinspires.ftc.teamcode.utils.ShooterMath;

import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;


@NextAutonomous(name = "Test AutoV2", preselectTeleop = "BIOBUZZ Teleop")
public class TestAutoV2 extends NextOpMode {
    private final Robot robot;


    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPose = poseFactory.of(56.7232, 8, 90);
    private final Pose point1 = poseFactory.of(7.7888, 8.2785, 180);
    private final Pose point2 = poseFactory.of(26.8284, 13.4357, -164.8444);
    private final Pose point3 = poseFactory.of(59.2785, 117.776, -160.1785);
    private final Pose point3Control1 = poseFactory.of(22.9003, 104.6636, 0);
    private final Pose point4 = poseFactory.of(47.3426, 129.872, 134.6183);
    private final Pose point5 = poseFactory.of(47.316, 138.5145, 90.1765);
    private final Pose point6 = poseFactory.of(47.6807, 129.3912, 92.2888);
    private final Pose point7 = poseFactory.of(59.448, 117.4881, 134.6715);
    private final Pose point8 = poseFactory.of(5.0156, 103.9171, -166.0005);

    public TestAutoV2(Robot robot) {
        super(robot);
        this.robot = robot;

        robot.drivetrain.initialize(hardwareMap);
        robot.turret.initialize(hardwareMap);

        robot.drivetrain.setPose(startPose);
    }

    @Override
    public void start() {
        RobotState.SOTM = false;

        sequential(
                robot.intake.closeGate(),
                robot.flywheel.enable(),
                robot.turret.enableTracking(),

                // preloads
                //shoot
                
                robot.intake.run(),
                robot.drivetrain.follow(path1()),
                robot.drivetrain.follow(path2()),
                robot.intake.stop(),
                robot.drivetrain.follow(path3()),
                //shoot,

                robot.drivetrain.follow(path4()),
                robot.intake.run(),
                robot.drivetrain.follow(path5()),
                robot.drivetrain.follow(path6()),
                robot.intake.stop(),
                robot.drivetrain.follow(path7()),
                //shoot,
                
                shutdown(),
                robot.drivetrain.follow(path8())
        ).schedule();
    }
    
    private Command shutdown() {
        return sequential(
                robot.intake.stop(),
                robot.flywheel.disable(),
                robot.turret.disableTracking()
        );
    }

    @Override
    public void periodic() {
        Pose pose = robot.drivetrain.getPose();
        ShooterMath.ShotSolution shot = robot.flywheel.getShotSolution();

        telemetry.addData("X", pose.x());
        telemetry.addData("Y", pose.y());
        telemetry.addData("Heading", Math.toDegrees(pose.heading()));

        //telemetry.addData("Ready To Shoot", robot.
        telemetry.addData("Shot Valid", shot.valid);
        telemetry.addData("Distance", shot.horizontalDistance);
        telemetry.addData("Flywheel Enabled", robot.flywheel.isFlywheelEnabled());
        telemetry.addData("Target RPM", robot.flywheel.getTargetVelocity());
        telemetry.addData("Current RPM", robot.flywheel.getCurrentVelocity());
        telemetry.addData("At Speed", robot.flywheel.isAtSpeed());
        telemetry.addData("Hood Position", robot.flywheel.getTargetHoodPosition());
        telemetry.addData("Turret Target", robot.turret.getTargetPosition());
        telemetry.addData("Turret Pos", robot.turret.getTurretPosition());
        telemetry.addData("Turret At Target", robot.turret.isAtTarget());

        telemetry.update();
    }

    // path 1, constant heading (holds 180 the whole time)
    private Path path1() {
        return Paths.line(startPose, point1).constant(point1);
    }

    // path 2, straight line, reverse tangent (backwards)
    private Path path2() {
        return Paths.line(point1, point2).reverseTangent();
    }

    // path 3, curve, reverse tangent (backwards)
    private Path path3() {
        return Paths.curve(point2, point3Control1, point3).reverseTangent();
    }

    // path 4, straight line, tangent (forwards)
    private Path path4() {
        return Paths.line(point3, point4).tangent();
    }

    // path 5, straight line, tangent (forwards)
    private Path path5() {
        return Paths.line(point4, point5).tangent();
    }

    // path 6, straight line, reverse tangent (backwards)
    private Path path6() {
        return Paths.line(point5, point6).reverseTangent();
    }

    // path 7, straight line, reverse tangent (backwards)
    private Path path7() {
        return Paths.line(point6, point7).reverseTangent();
    }

    // path 8, straight line, tangent (forwards)
    private Path path8() {
        return Paths.line(point7, point8).tangent();
    }
}
