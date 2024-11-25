package org.firstinspires.ftc.teamcode.auto.paths.old;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class FINALS {
    public static PathChain path() {
        PathBuilder builder = new PathBuilder();
        builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(9.000, 87.000, Point.CARTESIAN),
                                new Point(15.729, 112.985, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(15.729, 112.985, Point.CARTESIAN),
                                new Point(17.058, 124.505, Point.CARTESIAN),
                                new Point(13.735, 129.600, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Point(13.735, 129.600, Point.CARTESIAN),
                                new Point(23.926, 111.212, Point.CARTESIAN),
                                new Point(36.332, 108.111, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(36.332, 108.111, Point.CARTESIAN),
                                new Point(66.240, 104.123, Point.CARTESIAN),
                                new Point(60.037, 120.074, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(60.037, 120.074, Point.CARTESIAN),
                                new Point(27.914, 117.637, Point.CARTESIAN),
                                new Point(13.735, 129.600, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .addPath(
                        // Line 6
                        new BezierCurve(
                                new Point(13.735, 129.600, Point.CARTESIAN),
                                new Point(50.068, 106.338, Point.CARTESIAN),
                                new Point(89.945, 123.397, Point.CARTESIAN),
                                new Point(54.277, 129.822, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(54.277, 129.822, Point.CARTESIAN),
                                new Point(13.735, 129.600, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .addPath(
                        // Line 8
                        new BezierLine(
                                new Point(13.735, 129.600, Point.CARTESIAN),
                                new Point(32.788, 35.446, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                .addPath(
                        // Line 9
                        new BezierCurve(
                                new Point(32.788, 35.446, Point.CARTESIAN),
                                new Point(46.745, 38.548, Point.CARTESIAN),
                                new Point(45.858, 30.351, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
                .addPath(
                        // Line 10
                        new BezierLine(
                                new Point(45.858, 30.351, Point.CARTESIAN),
                                new Point(10.634, 34.117, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-180));
        return builder.build();
    }
}
