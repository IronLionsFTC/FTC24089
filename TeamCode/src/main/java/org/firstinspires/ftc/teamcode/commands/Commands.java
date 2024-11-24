package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

public class Commands {
    // Pedro Pathing
    public static Command followPath(Follower follower, PathChain path) { return new FollowPathCommand(follower, path); }
    public static Command followPath(Follower follower, Path path) { return new FollowPathCommand(follower, path); }
}
