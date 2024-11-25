package org.firstinspires.ftc.teamcode.auto.paths.old;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class PushThenParkWithDodge {
    public static PathChain path() {
        PathBuilder builder = new PathBuilder();
        builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(9.000, 87.000, Point.CARTESIAN),
                                new Point(15.000, 110.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(15.000, 110.000, Point.CARTESIAN),
                                new Point(19.052, 123.397, Point.CARTESIAN),
                                new Point(13.292, 130.265, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(13.292, 130.265, Point.CARTESIAN),
                                new Point(32.788, 96.369, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(32.788, 96.369, Point.CARTESIAN),
                                new Point(42.314, 18.166, Point.CARTESIAN),
                                new Point(10.191, 25.255, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90));
        return builder.build();
    }
}
