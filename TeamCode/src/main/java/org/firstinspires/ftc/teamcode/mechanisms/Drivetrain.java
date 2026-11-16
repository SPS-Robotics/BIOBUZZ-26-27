package org.firstinspires.ftc.teamcode.mechanisms;

import org.firstinspires.ftc.teamcode.globals.Constants;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.robot.Mechanism;

import com.pedropathing.paths.Path;

public class Drivetrain implements Mechanism {

    private Follower follower;

    private double driveHeadingOffset = 0.0;

    private double squareInput(double input) {
        return input * input * Math.signum(input);
    }

    public void initialize(HardwareMap hardwareMap) {
        follower =
                org.firstinspires.ftc.teamcode.pedro.Constants.create(
                        hardwareMap
                );
    }


    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public Pose getPose() {
        return follower.pose();
    }

    public double getAngularVelocity() {
        return follower.velocity().omega;
    }

    public double getVelocityX() {
        return follower.velocity().vx;
    }

    public double getVelocityY() {
        return follower.velocity().vy;
    }


    public Command resetHeading() {
        return Commands.instant(() ->
                driveHeadingOffset = follower.pose().heading()
        );
    }


    public void startDrive(Gamepad gamepad) {

        infinite(() -> {

            double forward = squareInput(-gamepad.left_stick_y);
            double lateral = squareInput(gamepad.left_stick_x);
            double turn = squareInput(gamepad.right_stick_x);

            double scalar =
                    gamepad.left_trigger > Constants.Drivetrain.SLOW_MODE_TRIGGER_THRESHOLD ? Constants.Drivetrain.SLOW_MODE_SCALAR : 1.0;

            DrivePowers powers = ManualDrive.fieldCentric(
                    forward * scalar,
                    lateral * scalar,
                    turn * scalar,
                    follower.pose().heading() - driveHeadingOffset
            );

            follower.manual(powers);

        }).schedule();
    }


    @Override
    public void periodic() {
        if (follower != null) {
            follower.update();
        }
    }

    public Command followPath(Path path) {
        return com.pedropathing.ivy.pedro.PedroCommands.follow(
                follower,
                path
        );
    }
}