package org.firstinspires.ftc.teamcode.auto.paths.old.yellow_spike;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import org.firstinspires.ftc.teamcode.auto.constants.Points;

public class BOTTOM_return {
    public static PathChain path() {
        PathBuilder builder = new PathBuilder();

        builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(30.0, 121.4, Point.CARTESIAN),
                                Points.basketOuttake
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45));

        return builder.build();
    }
}
