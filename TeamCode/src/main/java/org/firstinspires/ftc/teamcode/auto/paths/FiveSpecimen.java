package org.firstinspires.ftc.teamcode.auto.paths;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class FiveSpecimen {
    private static Point pointmm(double x, double y) { return new Point(x/25.4,y/25.4,Point.CARTESIAN); }
    private static Point pointin(double x, double y) { return new Point(x,y,Point.CARTESIAN); }
    private static double rad(double deg) { return deg / 180 * Math.PI; }

    private static Point start = pointmm(0,0);
    private static Point dump1 = pointmm(650,330);
    private static Point int1 = pointmm(650,-400);
    private static Point hook1 = pointmm(1100, -400);
    private static Point push1 = pointmm(200, 1300);

    public static PathChain path() {
        PathBuilder builder = new PathBuilder();
        return builder
                .addPath(
                        new BezierLine(
                                start,
                                dump1
                        )
                ).setConstantHeadingInterpolation(0)
                .addPath(
                        new BezierLine(
                                dump1,
                                int1
                        )
                ).setLinearHeadingInterpolation(0,rad(90))
                .addPath(
                        new BezierLine(
                                int1,
                                hook1
                        )
                ).setLinearHeadingInterpolation(rad(90), rad(90))
                .addPath(
                        new BezierLine(
                                hook1,
                                push1
                        )
                ).setLinearHeadingInterpolation(rad(90),0)

                .build();
    }
}
