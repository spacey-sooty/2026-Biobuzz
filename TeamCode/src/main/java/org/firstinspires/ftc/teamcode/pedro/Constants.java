package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    // From the Mecanum tuner, matches subsystems/Drive.java
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backLeft");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    // No odometry pods, so position comes from the drive motor encoders.
    // TODO measure both with the "Drive Encoder Test" OpMode, these are guesses
    public static double forwardTicksPerInch = 42;
    public static double strafeTicksPerInch = 42;

    // TODO paste the Foresight tuner output here, these are Pedro's defaults
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {});

    public static Localizer localizer(HardwareMap h) {
        return new DriveEncoderLocalizer(h, drivetrainConfig, "expander", forwardTicksPerInch, strafeTicksPerInch);
    }

    public static Drivetrain drivetrain(HardwareMap h) {
        return new Mecanum(h, drivetrainConfig);
    }

    public static Follower create(HardwareMap h) {
        return new Follower(localizer(h), drivetrain(h), new Foresight(foresightConfig));
    }
}
