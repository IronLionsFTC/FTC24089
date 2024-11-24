package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.auto.paths.Paths;
import org.firstinspires.ftc.teamcode.commands.Commands;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

@Autonomous(name = "commandBasedSampleDemonstration", group = "Testing")
public class commandBasedSampleDemonstration extends CommandOpMode {
    public PathChain chain;
    public Follower follower;
    public AutonomousRobot robot;

    @Override
    public void initialize() {
        this.follower = new Follower(hardwareMap);
        this.robot = new AutonomousRobot(telemetry, hardwareMap, follower);
        this.follower.setStartingPose(new Pose(0.0, 0.0, 0.0));
        this.chain = Paths.specimenAutoTest; // This is also a valid path for a simple sample run, just forwards then back

        schedule(
            new RunCommand(robot::update),
            new SequentialCommandGroup(
                    new WaitUntilCommand(this::opModeIsActive),
                    Commands.ExtendIntakeToGripSample(robot),
                    Commands.GrabGameObjectWithIntake(robot),
                    Commands.RetractIntakeForTransfer(robot),
                    Commands.RaiseSlidesForSampleDump(robot),
                    Commands.DumpSample(robot),
                    new InstantCommand(this::terminateOpModeNow)
            )
        );
    }
}
