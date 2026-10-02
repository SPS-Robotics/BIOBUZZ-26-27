package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;

public class Constants {
    public static Follower create(HardwareMap h) {
        return new Follower
                (new PinpointLocalizer(h,localizerConfig), new Mecanum(h, drivetrainConfig), new Foresight(foresightConfig));
    }

    public static MecanumConfig drivetrainConfig =
            new MecanumConfig(c -> {

                c.frontLeftName.set("frontLeft");
                c.backLeftName.set("backLeft");
                c.frontRightName.set("frontRight");
                c.backRightName.set("backRight");

                c.frontLeftDirection.set(
                        DcMotorSimple.Direction.REVERSE
                );

                c.backLeftDirection.set(
                        DcMotorSimple.Direction.REVERSE
                );

                c.frontRightDirection.set(
                        DcMotorSimple.Direction.FORWARD
                );

                c.backRightDirection.set(
                        DcMotorSimple.Direction.FORWARD
                );

                c.manualBrakeMode.set(true);
            });

    public static PinpointConfig localizerConfig =
            new PinpointConfig(c -> {

                c.name.set("pinpoint");

                // i need to put the actual values
                c.xPodOffset.set(0.0);
                c.yPodOffset.set(0.0);

                c.xPodDirection.set(
                        GoBildaPinpointDriver.EncoderDirection.FORWARD
                );

                c.yPodDirection.set(
                        GoBildaPinpointDriver.EncoderDirection.FORWARD
                );
            });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {}
    );
}