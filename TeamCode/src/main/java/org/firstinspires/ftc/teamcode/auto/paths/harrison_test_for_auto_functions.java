package org.firstinspires.ftc.teamcode.auto.paths;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class harrison_test_for_auto_functions {
    public static PathChain path() {
        PathBuilder builder = new PathBuilder();

        builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(9.757, 84.983, Point.CARTESIAN),
                                new Point(38.549, 84.919, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(0.0, 0.0)
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(38.549, 84.919, Point.CARTESIAN),
                                new Point(15.0, 84.983, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(0.0, 0.785)
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(15.0, 84.983, Point.CARTESIAN),
                                new Point(38.549, 84.919, Point.CARTESIAN)
                        )
                    )
                .setLinearHeadingInterpolation(0.785, -0.125)
                .addPath(
                // Line 2
                new BezierLine(
                        new Point(38.549, 84.919, Point.CARTESIAN),
                        new Point(15.0, 84.983, Point.CARTESIAN)
                )
        )
                .setLinearHeadingInterpolation(-0.125, 0.785)
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(15.0, 84.983, Point.CARTESIAN),
                                new Point(38.549, 84.919, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(0.785, 0.125)
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(38.549, 84.919, Point.CARTESIAN),
                                new Point(15.0, 84.983, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(0.125, 0.785);
        return builder.build();
    }
}
