package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class DriveToPoint extends CommandBase {
    private final Follower follower;
    private final Path path;
    private boolean holdEnd = true;
    private double maxSpeed = 1;

    public DriveToPoint(Follower follower, Path path) {
        this.follower = follower;
        this.path = path;
    }

    public DriveToPoint(Follower f, Double relX, Double relY) {
        double x = f.getPose().getX();
        double y = f.getPose().getY();
        double tx = f.getPose().getX() + relX;
        double ty = f.getPose().getY() + relY;

        PathBuilder builder = new PathBuilder();
        Point start = new Point(x, y, Point.CARTESIAN);
        Point end = new Point(tx, ty, Point.CARTESIAN);
        BezierLine start_to_end = new BezierLine(
                start,
                end
        );
        BezierLine end_to_start = new BezierLine(
                end,
                start
        );

        builder
                .addPath(start_to_end).setLinearHeadingInterpolation(0.0, 0.0)
                .addPath(end_to_start).setLinearHeadingInterpolation(0.0, 0.0);
        path = builder.build().getPath(0);
        follower = f;
    }

    /**
     * Decides whether or not to make the robot maintain its position once the path ends.
     *
     * @param holdEnd If the robot should maintain its ending position
     * @return This command for compatibility in command groups
     */
    public DriveToPoint setHoldEnd(boolean holdEnd) {
        this.holdEnd = holdEnd;
        return this;
    }

    /**
     * Sets the follower's maximum speed
     * @param speed Between 0 and 1
     * @return This command for compatibility in command groups
     */
    public DriveToPoint setSpeed(double speed) {
        this.maxSpeed = speed;
        return this;
    }

    @Override
    public void initialize() {
        follower.setMaxPower(this.maxSpeed);
        follower.followPath(path, holdEnd);
    }

    @Override
    public boolean isFinished() {
        return follower.getCurrentTValue() > 0.99;
    }

    @Override
    public void end(boolean interrupted) {
        follower.setMaxPower(1);
    }
}