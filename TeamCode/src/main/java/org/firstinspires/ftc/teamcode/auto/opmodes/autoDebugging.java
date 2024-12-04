package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
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

@Autonomous(name = "DEBUGGING", group = "Complete")
public class autoDebugging extends CommandOpMode {
    public PathChain chain;
    public Follower follower;
    public AutonomousRobot robot;

    @Override
    public void initialize() {
        this.follower = new Follower(hardwareMap);
        this.robot = new AutonomousRobot(telemetry, hardwareMap, follower, Team.Red);
        this.follower.setStartingPose(new Pose(0.0, 0.0, 0.0));
        this.chain = Paths.debug;

        schedule(
                new RunCommand(robot::update),
                new SequentialCommandGroup(
                        Commands.sleepUntil(this::opModeIsActive),
                        Commands.ExtendIntakeToGripSample(robot),
                        /*
                        Commands.followPath(follower, this.chain.getPath(0)).setSpeed(0.3).raceWith(
                            Commands.LookForSampleForRaceCondition(robot)
                        ),
                         */
                        Commands.LookForSampleForRaceCondition(robot),
                        new InstantCommand(follower::breakFollowing),
                        new InstantCommand(robot::logRotation),
                        Commands.RotateClawToCache(robot),
                        Commands.DriveToCachedPoint(robot, follower),
                        Commands.sleep(1000),
                        Commands.GrabGameObjectWithIntake(robot)
                )
        );
    }
}
