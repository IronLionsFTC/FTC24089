package org.firstinspires.ftc.teamcode.auto.paths;

import org.firstinspires.ftc.teamcode.auto.constants.Points;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;

public class BasketStartPushToPark {
    public static PathChain path() {
        PathBuilder builder = new PathBuilder();

        builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(9.000, 90.000, Point.CARTESIAN),
                                new Point(12.000, 100.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(12.000, 100.000, Point.CARTESIAN),
                                new Point(13.500, 130.700, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(13.500, 130.700, Point.CARTESIAN),
                                new Point(32.345, 111.212, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(135))
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(32.345, 111.212, Point.CARTESIAN),
                                new Point(68.677, 115.422, Point.CARTESIAN),
                                new Point(65.575, 98.806, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(-90));

        return builder.build();
    }
}
