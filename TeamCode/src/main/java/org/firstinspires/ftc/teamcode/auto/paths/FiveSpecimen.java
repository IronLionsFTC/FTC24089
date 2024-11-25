package org.firstinspires.ftc.teamcode.auto.paths;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class FiveSpecimen {
    private static Point point(double x, double y) { return new Point(x,y,Point.CARTESIAN); }

    private static Point start = point(0,0);
    private static Point dump1 = point(-650,0);

    public static PathChain path() {
        return new PathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(8.500, 64.000, Point.CARTESIAN),
                                new Point(37.000, 64.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(37.000, 64.000, Point.CARTESIAN),
                                new Point(26.492, 47.296, Point.CARTESIAN),
                                new Point(37.000, 35.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-60))
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Point(37.000, 35.000, Point.CARTESIAN),
                                new Point(60.623, 28.605, Point.CARTESIAN),
                                new Point(12.500, 20.479, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-60), Math.toRadians(180))
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(12.500, 20.479, Point.CARTESIAN),
                                new Point(37.707, 24.054, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-60))
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(37.707, 24.054, Point.CARTESIAN),
                                new Point(63.874, 17.716, Point.CARTESIAN),
                                new Point(12.500, 17.065, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-60), Math.toRadians(180))
                .addPath(
                        // Line 6
                        new BezierCurve(
                                new Point(12.500, 17.065, Point.CARTESIAN),
                                new Point(62.411, 18.853, Point.CARTESIAN),
                                new Point(61.761, 8.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(61.761, 8.000, Point.CARTESIAN),
                                new Point(12.500, 8.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }
}
