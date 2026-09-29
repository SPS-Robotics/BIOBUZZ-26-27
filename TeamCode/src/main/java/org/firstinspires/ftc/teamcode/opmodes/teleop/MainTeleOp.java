package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;

import org.firstinspires.ftc.teamcode.Robot;

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

        // reset heading for feild centric view
        driver.start().onTrue(robot.drivetrain.resetHeading());


    }

}
