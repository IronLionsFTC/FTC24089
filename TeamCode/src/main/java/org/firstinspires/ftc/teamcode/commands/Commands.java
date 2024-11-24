package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

public class Commands {
    // Pedro Pathing
    public static Command followPath(Follower follower, PathChain path) { return new FollowPathCommand(follower, path); }
    public static Command followPath(Follower follower, Path path) { return new FollowPathCommand(follower, path); }

    // Intake Commands ---------------------------------------------------------------------------------------------------

    // Extend into various states
    public static Command ExtendIntakeToGripSample(AutonomousRobot robot) { return new ExtendIntakeToGripSample(robot); }
    public static Command ExtendIntakeToGripSpecimen(AutonomousRobot robot) { return new ExtendIntakeToGripSpecimen(robot); }

    // Sample & Specimen Grab Function
    public static Command GrabGameObjectWithIntake(AutonomousRobot robot) { return new GrabGameObjectWithIntake(robot); }

    // Retract the intake and end when a transfer is possible, doesn't actually transfer
    public static Command RetractIntakeForTransfer(AutonomousRobot robot) { return new RetractIntakeForTransfer(robot); }

    // Outtake Commands ---------------------------------------------------------------------------------------------------

    // First stage outtake commands
    public static Command RaiseSlidesForSampleDump(AutonomousRobot robot)  { return new RaiseSlidesForSampleDump(robot); }
    public static Command RaiseSlidesForSpecimenDump(AutonomousRobot robot) { return new RaiseSlidesForSpecimenDump(robot); }

    // Clip / dump commands, clip automatically lets go when reached to be idiot proof and not damage slides
    public static Command DumpSample(AutonomousRobot robot) { return new DumpSample(robot); }
    public static Command ClipSpecimen(AutonomousRobot robot) { return new ClipSpecimen(robot); }
}
