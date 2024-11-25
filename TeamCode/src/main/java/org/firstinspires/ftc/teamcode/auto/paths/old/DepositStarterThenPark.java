package org.firstinspires.ftc.teamcode.auto.paths.old;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class DepositStarterThenPark {
    public static PathChain path() {
        PathBuilder builder = new PathBuilder();
        builder
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(9.000, 45.000, Point.CARTESIAN),
                                new Point(8.862, 66.905, Point.CARTESIAN),
                                new Point(16.837, 71.557, Point.CARTESIAN),
                                new Point(16.837, 84.628, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(16.837, 84.628, Point.CARTESIAN),
                                new Point(16.837, 115.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-90))
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Point(16.837, 115.000, Point.CARTESIAN),
                                new Point(18.831, 122.068, Point.CARTESIAN),
                                new Point(17.502, 125.391, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(17.502, 125.391, Point.CARTESIAN),
                                new Point(22.375, 116.308, Point.CARTESIAN),
                                new Point(14.178, 100.357, Point.CARTESIAN),
                                new Point(14.178, 80.418, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(14.178, 80.418, Point.CARTESIAN),
                                new Point(14.178, 12.628, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(90));
        return builder.build();
    }
}