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
    private static BezierLine line(Point start, Point end) { return new BezierLine(start, end); }

    private static Point start = pointmm(0,0);
    private static Point dump1 = pointmm(670,300);
    private static Point int1 = pointmm(550,-400);
    private static Point hook1_1 = pointmm(1100, -400);
    private static Point hook1_2 = pointmm(1100, -550);
    private static Point push1 = pointmm(200, -1300);
    private static Point hook2 = pointmm(1100, -700);

    public static PathChain path() {
        PathBuilder builder = new PathBuilder();
        return builder
                // Preloaded specimen
                .addPath(
                        line(start, dump1)
                ).setConstantHeadingInterpolation(rad(180))
                // Push first floor
                .addPath(
                        line(dump1, int1)
                ).setLinearHeadingInterpolation(rad(180),rad(-90))
                .addPath(
                        line(int1, hook1_1)
                ).setLinearHeadingInterpolation(rad(-90), rad(-90))
                .addPath(
                        line(hook1_1, hook1_2)
                ).setLinearHeadingInterpolation(rad(-90), rad(-90))
                .addPath(
                        line(hook1_2, push1)
                ).setLinearHeadingInterpolation(rad(-90),rad(180))
                // Go to second
                .addPath(
                        line(push1, hook2)
                ).setLinearHeadingInterpolation(rad(180), rad(90))
                // Push second


                .build();
    }
}
