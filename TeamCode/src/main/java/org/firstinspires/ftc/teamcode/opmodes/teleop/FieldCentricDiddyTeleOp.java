package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Field Centric Diddy Teleop", group = "Test")
public class FieldCentricDiddyTeleOp extends LinearOpMode {

    private static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    private static final RevHubOrientationOnRobot.UsbFacingDirection USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");

        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(LOGO_DIRECTION, USB_DIRECTION)
        ));

        DcMotor intakeMotor = hardwareMap.tryGet(DcMotor.class, "intakeMotor");
        if (intakeMotor != null) {
            intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        }

        DcMotor turretMotor = hardwareMap.tryGet(DcMotor.class, "turretRotator");
        if (turretMotor != null) {
            turretMotor.setDirection(DcMotor.Direction.FORWARD);
            turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            turretMotor.setPower(0.0);
        }

        DcMotor flywheelMotor1 = hardwareMap.tryGet(DcMotor.class, "flywheelMotor1");
        DcMotor flywheelMotor2 = hardwareMap.tryGet(DcMotor.class, "flywheelMotor2");

        if (flywheelMotor1 != null) {
            flywheelMotor1.setDirection(DcMotor.Direction.REVERSE);
            flywheelMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            flywheelMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            flywheelMotor1.setPower(0.0);
        }
        if (flywheelMotor2 != null) {
            flywheelMotor2.setDirection(DcMotor.Direction.FORWARD);
            flywheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            flywheelMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            flywheelMotor2.setPower(0.0);
        }

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotor motor : new DcMotor[]{frontLeft, backLeft, frontRight, backRight}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0.0);
        }

        double maxShooterPower = 1.0;
        double shooterPower = 0.0;
        ElapsedTime shooterTimer = new ElapsedTime();
        boolean previousOptions = false;

        telemetry.addData("Status", "Ready");
        telemetry.addData("IMU", "imu");
        telemetry.addData("Flywheel 1 (bottom)", flywheelMotor1 != null ? "Configured" : "Missing");
        telemetry.addData("Flywheel 2 (top)", flywheelMotor2 != null ? "Configured" : "Missing");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        imu.resetYaw();
        shooterTimer.reset();

        try {
            while (opModeIsActive()) {
                boolean options = gamepad1.options;
                if (options && !previousOptions) {
                    imu.resetYaw();
                }
                previousOptions = options;

                double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                double y = gamepad1.left_stick_y;
                double x = -gamepad1.left_stick_x;
                double rx = -0.5 * gamepad1.right_stick_x;

                double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
                double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

                double denominator = Math.max(
                        Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0
                );
                double speed = gamepad1.left_trigger > 0.1 ? 0.20 : 1.0;

                frontLeft.setPower((rotY + rotX + rx) / denominator * speed);
                backLeft.setPower((rotY - rotX + rx) / denominator * speed);
                frontRight.setPower((rotY - rotX - rx) / denominator * speed);
                backRight.setPower((rotY + rotX - rx) / denominator * speed);

                if (intakeMotor != null) {
                    double intakePower = 0.0;
                    if (gamepad1.cross) {
                        intakePower = -0.6;
                    } else if (gamepad1.right_trigger > 0.1) {
                        intakePower = 1.0;
                    }
                    intakeMotor.setPower(intakePower);
                }

                if (turretMotor != null) {
                    double turretPower = 0.0;
                    if (gamepad1.dpad_left && !gamepad1.dpad_right) {
                        turretPower = -0.10;
                    } else if (gamepad1.dpad_right && !gamepad1.dpad_left) {
                        turretPower = 0.10;
                    }
                    turretMotor.setPower(turretPower);
                }

                double dt = Math.min(shooterTimer.seconds(), 0.1);
                shooterTimer.reset();

                if (gamepad1.square) {
                    shooterPower = 0.0;
                } else if (gamepad1.left_bumper) {
                    shooterPower = Math.min(maxShooterPower, shooterPower + dt * 0.5);
                }

                if (flywheelMotor1 != null) {
                    flywheelMotor1.setPower(shooterPower);
                }
                if (flywheelMotor2 != null) {
                    flywheelMotor2.setPower(shooterPower);
                }

                telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(heading));
                telemetry.addData("Shooter Power", "%.2f", shooterPower);
                telemetry.addData("Intake", gamepad1.right_trigger > 0.1);
                telemetry.addData("Reset Heading", "Options");
                telemetry.update();
            }
        } finally {
            frontLeft.setPower(0.0);
            backLeft.setPower(0.0);
            frontRight.setPower(0.0);
            backRight.setPower(0.0);
            if (intakeMotor != null) intakeMotor.setPower(0.0);
            if (turretMotor != null) turretMotor.setPower(0.0);
            if (flywheelMotor1 != null) flywheelMotor1.setPower(0.0);
            if (flywheelMotor2 != null) flywheelMotor2.setPower(0.0);
        }
    }
}
