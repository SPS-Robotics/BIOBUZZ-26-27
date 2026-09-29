package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;

import java.util.Set;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;

public class Robot implements NextRobot {

    public final Intake intake = new Intake();
    public final Drivetrain drivetrain = new Drivetrain();
    public final Flywheel flywheel = new Flywheel(drivetrain);

    public final Turret turret = new Turret(drivetrain);

    @Override
    public Set<Mechanism> getMechanisms() {
        return Set.of(intake, drivetrain, flywheel, turret);
    }


}
