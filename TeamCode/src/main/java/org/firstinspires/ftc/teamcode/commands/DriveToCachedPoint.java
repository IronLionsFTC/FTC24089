package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class DriveToCachedPoint extends CommandBase {
    private final Follower follower;
    private boolean holdEnd = true;
    private double maxSpeed = 1;
    private AutonomousRobot robot;

    public DriveToCachedPoint(AutonomousRobot r, Follower f) {
        robot = r;
        follower = f;
    }

    /**
     * Decides whether or not to make the robot maintain its position once the path ends.
     *
     * @param holdEnd If the robot should maintain its ending position
     * @return This command for compatibility in command groups
     */
    public DriveToCachedPoint setHoldEnd(boolean holdEnd) {
        this.holdEnd = holdEnd;
        return this;
    }

    /**
     * Sets the follower's maximum speed
     * @param speed Between 0 and 1
     * @return This command for compatibility in command groups
     */
    public DriveToCachedPoint setSpeed(double speed) {
        this.maxSpeed = speed;
        return this;
    }

    @Override
    public void initialize() {
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        double r = follower.getPose().getHeading();

        double rx = robot.sampleY * Math.cos(r) + robot.sampleX * Math.sin(r);
        double ry = robot.sampleX * Math.cos(r) + robot.sampleY * Math.sin(r);

        double tx = x + rx;
        double ty = y - ry;

        PathBuilder builder = new PathBuilder();
        builder.addPath(
                new BezierLine(
                        new Point(x, y, 1),
                        new Point(tx, ty, 1)
                )
        //).setConstantHeadingInterpolation(Math.toRadians(100));
        ).setConstantHeadingInterpolation(follower.getPose().getHeading());
        follower.breakFollowing();
        follower.followPath(builder.build(), true);
        follower.setMaxPower(this.maxSpeed);
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