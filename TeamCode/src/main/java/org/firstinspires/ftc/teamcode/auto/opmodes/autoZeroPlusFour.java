package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.auto.paths.Paths;
import org.firstinspires.ftc.teamcode.commands.Commands;
import org.firstinspires.ftc.teamcode.core.state.Team;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

@Autonomous(name = "0+4", group = "Complete")
public class autoZeroPlusFour extends CommandOpMode {
    public PathChain chain;
    public Follower follower;
    public AutonomousRobot robot;

    @Override
    public void initialize() {
        this.follower = new Follower(hardwareMap);
        this.robot = new AutonomousRobot(telemetry, hardwareMap, follower, Team.Blue);
        this.follower.setStartingPose(new Pose(0.0, 0.0, 0.0));
        this.chain = Paths.zeroPlusFour; // This is also a valid path for a simple sample run, just forwards then back

        schedule(
            new RunCommand(robot::update),
            new SequentialCommandGroup(
                    new WaitUntilCommand(this::opModeIsActive),
                    Commands.followPath(follower, chain.getPath(0)).alongWith(
                            Commands.RaiseSlidesForSampleDumpAndFlip(robot)
                    ),
                    Commands.DropSample(robot),
                    Commands.followPath(follower, chain.getPath(1)).alongWith(
                            Commands.ExtendIntakeToGripSample(robot)
                    ),
                    Commands.sleep(1000), // Adjust over top of sample before grab
                    Commands.GrabGameObjectWithIntake(robot),
                    Commands.RetractIntakeForTransfer(robot).alongWith(
                            Commands.followPath(follower, chain.getPath(2))
                    ).andThen(
                        Commands.RaiseSlidesForSampleDump(robot)
                    ),
                    Commands.DumpSample(robot),
                    Commands.followPath(follower, chain.getPath(3)).alongWith(
                            Commands.ExtendIntakeToGripSample(robot)
                    ),
                    Commands.sleep(1000), // Adjust over top of sample before grab
                    Commands.GrabGameObjectWithIntake(robot),
                    Commands.RetractIntakeForTransfer(robot).alongWith(
                            Commands.followPath(follower, chain.getPath(4))
                    ).andThen(
                            Commands.RaiseSlidesForSampleDump(robot)
                    ),
                    Commands.DumpSample(robot),
                    Commands.followPath(follower, chain.getPath(5)).alongWith(
                            Commands.ExtendIntakeToGripSample(robot)
                    ),
                    Commands.RotateClaw45Degrees(robot),
                    Commands.sleep(1000), // Adjust over top of sample before grab
                    Commands.GrabGameObjectWithIntake(robot),
                    Commands.RetractIntakeForTransfer(robot).alongWith(
                            Commands.followPath(follower, chain.getPath(6))
                    ).andThen(
                            Commands.RaiseSlidesForSampleDump(robot)
                    ),
                    Commands.DumpSample(robot),
                    Commands.followPath(follower, chain.getPath(7)),
                    Commands.followPath(follower, chain.getPath(8))
            )
        );
    }
}
