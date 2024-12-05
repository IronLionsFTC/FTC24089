package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.auto.paths.Paths;
import org.firstinspires.ftc.teamcode.commands.Commands;
import org.firstinspires.ftc.teamcode.core.state.Team;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

@Autonomous(name = "0+5 CV [RED]", group = "Complete")
public class autoZeroPlusFiveRedNew extends CommandOpMode {
    public PathChain chain;
    public Follower follower;
    public AutonomousRobot robot;

    @Override
    public void initialize() {
        this.follower = new Follower(hardwareMap);
        this.robot = new AutonomousRobot(telemetry, hardwareMap, follower, Team.Red);
        this.follower.setStartingPose(new Pose(0.0, 0.0, 0.0));
        this.chain = Paths.zeroPlusFour; // This is also a valid path for a simple sample run, just forwards then back

        schedule(
            new RunCommand(robot::update),
            new SequentialCommandGroup(
                    new WaitUntilCommand(this::opModeIsActive),
                    Commands.sleep(100),
                    Commands.followPath(follower, chain.getPath(0)).alongWith(
                            Commands.RaiseSlidesForSampleDump(robot)
                    ),
                    Commands.sleep(500),
                    Commands.DumpSample(robot),
                    Commands.followPath(follower, chain.getPath(1)).alongWith(
                            Commands.ExtendIntakeToGripSample(robot)
                    ),
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
                    Commands.GrabGameObjectWithIntake(robot),
                    Commands.RetractIntakeForTransfer(robot).alongWith(
                            Commands.followPath(follower, chain.getPath(6))
                    ).andThen(
                            Commands.RaiseSlidesForSampleDump(robot)
                    ),
                    Commands.DumpSample(robot),
                    Commands.fastPath(follower, chain.getPath(7)),
                    Commands.ExtendIntakeToGripSample(robot),
                    Commands.followPath(follower, chain.getPath(8)).setSpeed(0.4).andThen(
                            Commands.followPath(follower, chain.getPath(9)).setSpeed(0.25)
                    ).raceWith(
                            Commands.LookForSampleForRaceCondition(robot, follower)
                    ),


                    // CV
                    Commands.RotateClawToCache(robot),
                    Commands.DriveToCachedPoint(robot, follower),
                    Commands.sleep(500),


                    Commands.GrabGameObjectWithIntake(robot),
                    Commands.RetractIntakeForTransfer(robot),
                    Commands.followPath(follower, chain.getPath(10)).alongWith(
                            Commands.RaiseSlidesForSampleDump(robot)
                    ),
                    Commands.DumpSample(robot),
                    Commands.followPath(follower, chain.getPath(7))
            )
        );
    }
}
