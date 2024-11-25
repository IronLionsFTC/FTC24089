package org.firstinspires.ftc.teamcode.auto.paths;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class ZeroPlusFour {
    public static PathChain path() {
        PathBuilder builder = new PathBuilder();
        builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(0.0, 0.0, Point.CARTESIAN),
                                new Point(-2.0, -36.0, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-235))
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(-2.0, -36.0, Point.CARTESIAN),
                                new Point(-7.5, -31.0, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-235), Math.toRadians(-180))
                .addPath(
                        new BezierLine(
                                new Point(-7.5, -29.0, Point.CARTESIAN),
                                new Point(-2.0, -36.0, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-235))
                .addPath(
                        new BezierLine(
                                new Point(-2.0, -36.0, Point.CARTESIAN),
                                new Point(-7.5, -39.0, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-235), Math.toRadians(-180))
                .addPath(
                        new BezierLine(
                                new Point(-7.5, -39.0, Point.CARTESIAN),
                                new Point(-2.0, -36.0, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-235))
                .addPath(
                        new BezierLine(
                                new Point(-2.0, -36.0, Point.CARTESIAN),
                                new Point(-14.0, -34.0, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-235), Math.toRadians(-135))
                .addPath(
                        new BezierLine(
                                new Point(-14.0, -34.0, Point.CARTESIAN),
                                new Point(-2.0, -36.0, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(-235))
                .addPath(
                        new BezierLine(
                                new Point(-2.0, -36.0, Point.CARTESIAN),
                                new Point(-5.0, 10.0, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-270))
                .addPath(
                        new BezierLine(
                                new Point(-5.0, 10.0, Point.CARTESIAN),
                                new Point(-5.0, 20.0, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-270), Math.toRadians(-630))
                .addPath(
                        new BezierLine(
                                new Point(-5.0, 10.0, Point.CARTESIAN),
                                new Point(1.0, 50.0, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-630))
        ;
        return builder.build();
    }
}