package org.firstinspires.ftc.teamcode;
import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.RunCommand;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous
public class Autos extends CommandOpMode {
    private static final double DRIVE_POWER = 0.4;
    private static final double Sh = 1300;
    private static final double FEED_POWER = 0.5;

    private static final long SPIN_UP_MILLIS = 1000;
    private static final long SHOOT_MILLIS = 4000;

    // OpModes need a no-arg constructor, so everything is created in initialize()
    private Drive drive;
    private Indexer indexer;
    private Shooter shooter;
    private Intake intake;
    private Follower follower;

//    Poses
    private final PoseFactory p = PoseFactory.degrees();

    public Command testingSequence() {
        return Commands.parallel(
                drive.bl(1).withTimeout(1),
                drive.bl(1).withTimeout(1),
                drive.bl(1).withTimeout(1),
                drive.bl(1).withTimeout(1),
                intake.run(1),
                Commands.waitMillis(2000),
                shooter.shoot(indexer).withTimeout(3000));
    }

    // Pedro poses are in inches and radians, x is forward and y is left of where the robot starts.
    public Command pathAuto() {
        Pose start = new Pose(136.2, 15, Math.toRadians(180));
        Pose point2 = new Pose(56, 15, Math.toRadians(180));
        Pose point3 = new Pose(56, 15, Math.toRadians(90));
        Pose shoot = new Pose(56, 7.8, Math.toRadians(90));

        Path part1 = line(start, point2).linear(start, point2);
        Path part2 = line(point3, shoot).linear(point3, shoot);

        return Commands.sequence(
                drive.runOnce(() -> follower.setPose(start)),
                intake.run(1),
                followPath(part1),
                followPath(part2),
                shooter.shoot(indexer));
    }

    // Only starts the path and waits for it, the OpMode has to call follower.update() every loop.
    private Command followPath(Path path) {
        return new FunctionalCommand(
                () -> follower.follow(path),
                () -> {},
                interrupted -> {
                    if (interrupted) follower.stop();
                },
                () -> !follower.following(),
                drive);
    }

    @Override
    public void initialize() {
        drive = new Drive(hardwareMap);
        indexer = new Indexer(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);

        follower = Constants.create(hardwareMap);
        if (follower == null)
            throw new IllegalStateException("Constants.create() returned null, fill in pedro/Constants.java with your tuned values");

        // follower has to update every loop for paths to run
        schedule(new RunCommand(follower::update), pathAuto());

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
}
