/*
  Things to Fix
 Constants Problems in globals constants
 -    flywheel
  -    hood
  -    turret
  -    GOAL_POSE
  - maybe more i'm not entirely sure
  - Timing (30s only) have to check if the timing actually works, like pretty much everything else this just needs testing on a correct game field.
  - Red side only. Paths 1, 5, and 8 shoot preloads only (no intake).
  - Correct size Bot? adjust based on size of bot, rn its 16"
 */
package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Robot;

import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;


@NextAutonomous(name = "Test Auto", preselectTeleop = "BIOBUZZ Teleop")
public class TestAuto extends NextOpMode {

    // max time to wait for flywheel + turret before shooting anyway
    private static final double SPIN_UP_TIMEOUT_MS = 2000;
    // how long the gate stays open and the intake feeds
    private static final double FEED_TIME_MS = 1500;

    private final Robot robot;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPose = poseFactory.of(56.7232, 8, 180);
    private final Pose point1 = poseFactory.of(10, 10, 180);
    private final Pose point2 = poseFactory.of(26.8284, 13.4357, -164.8444);
    private final Pose point3 = poseFactory.of(59.2785, 117.776, 19.8215);
    private final Pose point3Control1 = poseFactory.of(22.9003, 104.6636, 0);
    private final Pose point4 = poseFactory.of(47.0366, 117.8774, -0.4747);
    private final Pose point5 = poseFactory.of(47.316, 134, 89.2244);
    private final Pose point6 = poseFactory.of(59.448, 117.4881, 119.9845);
    private final Pose point7 = poseFactory.of(20, 46.6145, -108.8206);
    private final Pose point7Control1 = poseFactory.of(31.9242, 108.4617, 0);
    private final Pose point8 = poseFactory.of(10, 46.8902, 178.3449);
    private final Pose point9 = poseFactory.of(20, 46.6686, 178.7009);
    private final Pose point10 = poseFactory.of(57.2411, 23.2896, -26.8579);
    private final Pose point11 = poseFactory.of(14.0, 105.8458, -68.3235);
    private final Pose point11Control1 = poseFactory.of(31.4199, 39.8305, 0);
    private final Pose point12 = poseFactory.of(7.8, 105.8458, -68.3235);

    public TestAuto(Robot robot) {
        super(robot);
        this.robot = robot;

        robot.drivetrain.initialize(hardwareMap);
        robot.turret.initialize(hardwareMap);

        robot.drivetrain.setPose(startPose);
    }

    @Override
    public void start() {
        sequential(
                parallel(
                        robot.flywheel.enable(),
                        robot.turret.enableTracking()
                ),
                robot.drivetrain.follow(path1()),
                robot.drivetrain.follow(path2()),
                robot.drivetrain.follow(path3()),
                shoot(),
                robot.drivetrain.follow(path4()),
                robot.drivetrain.follow(path5()),
                robot.drivetrain.follow(path6()),
                shoot(),
                robot.drivetrain.follow(path7()),
                robot.drivetrain.follow(path8()),
                robot.drivetrain.follow(path9()),
                robot.drivetrain.follow(path10()),

                shoot(),
                robot.drivetrain.follow(path11()),
                robot.drivetrain.follow(path12()),
                robot.flywheel.disable()
        ).schedule();
    }

    // wait until ready (or timeout), then open the gate and feed everything through
    private Command shoot() {
        return sequential(
                race(
                        waitUntil(robot::readyToShoot),
                        waitMs(SPIN_UP_TIMEOUT_MS)
                ),
                robot.intake.openGate(),
                robot.intake.run(),
                waitMs(FEED_TIME_MS),
                robot.intake.stop(),
                robot.intake.closeGate()
        );
    }

    @Override
    public void periodic() {
        Pose pose = robot.drivetrain.getPose();

        telemetry.addData("X", pose.x());

        telemetry.addData("Y", pose.y());
        telemetry.addData("Heading", Math.toDegrees(pose.heading()));
    }

    // path 1, constant heading (holds 180 the wholeIt  time°)
    private Path path1() {
        return Paths.line(startPose, point1).constant(point1);
    }

    // path 2, straight line, reverse tangent (backwards)
    private Path path2() {
        return Paths.line(point1, point2).reverseTangent();
    }

    // path 3, curve, tangent (Faces where it's going)
    private Path path3() {
        return Paths.curve(point2, point3Control1, point3).tangent();
    }

    // path 4, straight line, reverse tangent (backwards)
    private Path path4() {
        return Paths.line(point3, point4).reverseTangent();
    }

    // path 5, straight line, tangent (faces where it's going)
    private Path path5() {
        return Paths.line(point4, point5).tangent();
    }

    // path 6, straight line, reverse tangent (backwards)
    private Path path6() {
        return Paths.line(point5, point6).reverseTangent();
    }

    // path 7, curve, tangent (faces where it's going)
    private Path path7() {
        return Paths.curve(point6, point7Control1, point7).tangent();
    }

    // path 8, straight line, tangent (faces where it's going)
    private Path path8() {
        return Paths.line(point7, point8).tangent();
    }

    // path 9, straight line, reverse tangent backwards
    private Path path9() {
        return Paths.line(point8, point9).reverseTangent();
    }

    // path 10, straight line, tangent (faces where it's going)
    private Path path10() {
        return Paths.line(point9, point10).tangent();
    }

    // path 11, curve, reverse tangent backwards
    private Path path11() {
        return Paths.curve(point10, point11Control1, point11).reverseTangent();
    }

    // path 12, short straight line into park, constant heading
    private Path path12() {
        return Paths.line(point11, point12).constant(point12);
    }
}
