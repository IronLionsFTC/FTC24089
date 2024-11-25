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
    private static Point dump1 = pointmm(650,300);
    private static Point int1 = pointmm(550,-650);
    private static Point hook1_1 = pointmm(1100, -650);
    private static Point hook1_2 = pointmm(1100, -770);
    private static Point push1 = pointmm(200, -770);
    private static Point hook2_1 = pointmm(1100, -920);
    private static Point hook2_2 = pointmm(1100, -1070);
    private static Point push2 = pointmm(200, -1070);
    private static Point hook3_control = pointmm(1100, -1070);
    private static Point hook3_end = pointmm(1100, -1330);
    private static Point push3 = pointmm(200, -1330);

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
                ).setConstantHeadingInterpolation(rad(180))
                .addPath(
                        line(int1, hook1_1)
                ).setLinearHeadingInterpolation(rad(180), rad(-90))
                .addPath(
                        line(hook1_1, hook1_2)
                ).setConstantHeadingInterpolation(rad(-90))
                .addPath(
                        line(hook1_2, push1)
                ).setLinearHeadingInterpolation(rad(-90),rad(180))
                // Go to second
                .addPath(
                        line(push1, hook2_1)
                ).setLinearHeadingInterpolation(rad(180), rad(-90))
                // Push second
                .addPath(
                        line(hook2_1, hook2_2)
                ).setConstantHeadingInterpolation(rad(-90))
                .addPath(
                        line(hook2_2, push2)
                ).setLinearHeadingInterpolation(rad(-90), rad(180))
                // Go to third
                .addPath(
                        new BezierCurve(
                                push2,
                                hook3_control,
                                hook3_end
                        )
                ).setConstantHeadingInterpolation(rad(180))
                // Push third
                .addPath(
                        line(hook3_end, push3)
                ).setConstantHeadingInterpolation(rad(180))


                .build();
    }
}
