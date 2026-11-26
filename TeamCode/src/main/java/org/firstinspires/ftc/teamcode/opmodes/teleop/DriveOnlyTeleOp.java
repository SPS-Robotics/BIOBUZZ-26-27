
package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "BIOBUZZ Drive Only", group = "Test")
public class DriveOnlyTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");

        DcMotor intakeMotor = hardwareMap.tryGet(
                DcMotor.class, "intakeMotor"
        );

        if (intakeMotor != null) {
            intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        }


        DcMotor turretMotor = hardwareMap.tryGet(
                DcMotor.class, "turretRotator"
        );

        if (turretMotor != null) {
            turretMotor.setDirection(DcMotor.Direction.FORWARD);
            turretMotor.setZeroPowerBehavior(
                    DcMotor.ZeroPowerBehavior.BRAKE
            );
            turretMotor.setMode(
                    DcMotor.RunMode.RUN_WITHOUT_ENCODER
            );
            turretMotor.setPower(0.0);
        }


        DcMotor flywheelMotor1 = hardwareMap.tryGet(
                DcMotor.class, "flywheelMotor1"
        );

        DcMotor flywheelMotor2 = hardwareMap.tryGet(
                DcMotor.class, "flywheelMotor2"
        );

        double maxShooterPower = 1.0;
        double shooterPower = 0.0;
        ElapsedTime shooterTimer = new ElapsedTime();

        if (flywheelMotor1 != null && flywheelMotor2 != null) {

            flywheelMotor1.setDirection(DcMotor.Direction.REVERSE);
            flywheelMotor2.setDirection(DcMotor.Direction.FORWARD);

            flywheelMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            flywheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            flywheelMotor1.setZeroPowerBehavior(
                    DcMotor.ZeroPowerBehavior.FLOAT
            );
            flywheelMotor2.setZeroPowerBehavior(
                    DcMotor.ZeroPowerBehavior.FLOAT
            );

            flywheelMotor1.setPower(0.0);
            flywheelMotor2.setPower(0.0);
        }

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Ready");
        telemetry.addData(
                "Flywheel Motors",
                flywheelMotor1 != null && flywheelMotor2 != null
                        ? "Connected" : "Missing"
        );
        telemetry.update();

        waitForStart();

        shooterTimer.reset();

        try {
            while (opModeIsActive()) {

                double y = -gamepad1.left_stick_y;
                double x = -gamepad1.left_stick_x;
                double rx = -gamepad1.right_stick_x;

                double denominator = Math.max(
                        Math.abs(y) + Math.abs(x) + Math.abs(rx),
                        1.0
                );

                double speed = gamepad1.left_trigger > 0.1
                        ? 0.20 : 1.0;

                frontLeft.setPower(
                        (y + x + rx) / denominator * speed
                );

                backLeft.setPower(
                        (y - x + rx) / denominator * speed
                );

                frontRight.setPower(
                        (y - x - rx) / denominator * speed
                );

                backRight.setPower(
                        (y + x - rx) / denominator * speed
                );

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

                if (flywheelMotor1 != null && flywheelMotor2 != null) {

                    if (gamepad1.left_bumper) {
                        shooterPower = Math.min(
                                maxShooterPower,
                                shooterPower + dt * 0.5
                        );
                    } else {
                        shooterPower = 0.0;
                    }

                    flywheelMotor1.setPower(shooterPower);
                    flywheelMotor2.setPower(shooterPower);
                }

                telemetry.addData("Shooter Power", shooterPower);
                telemetry.addData("Intake", gamepad1.right_trigger > 0.1);
                telemetry.update();
            }

        } finally {

            frontLeft.setPower(0.0);
            backLeft.setPower(0.0);
            frontRight.setPower(0.0);
            backRight.setPower(0.0);

            if (intakeMotor != null) {
                intakeMotor.setPower(0.0);
            }

            if (flywheelMotor1 != null) {
                flywheelMotor1.setPower(0.0);
            }

            if (flywheelMotor2 != null) {
                flywheelMotor2.setPower(0.0);
            }


            if (turretMotor != null) {
                turretMotor.setPower(0.0);
            }

        }
    }
}
