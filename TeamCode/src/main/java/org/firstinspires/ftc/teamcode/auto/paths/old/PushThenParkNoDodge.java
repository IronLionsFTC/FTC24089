package org.firstinspires.ftc.teamcode.auto.paths.old;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class PushThenParkNoDodge {
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
                                new Point(12.849, 62.252, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(12.849, 62.252, Point.CARTESIAN),
                                new Point(9.969, 35.889, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90));
        return builder.build();
    }
}
