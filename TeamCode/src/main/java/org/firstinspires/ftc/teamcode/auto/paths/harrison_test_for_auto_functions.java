package org.firstinspires.ftc.teamcode.auto.paths;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class harrison_test_for_auto_functions {
    public static PathChain path() {
        PathBuilder builder = new PathBuilder();
        Point start = new Point(0.0, 0.0, Point.CARTESIAN);
        Point end = new Point(25.0, 0.0, Point.CARTESIAN);
        BezierLine start_to_end = new BezierLine(
                start,
                end
        );
        BezierLine end_to_start = new BezierLine(
                end,
                start
        );

        builder
                .addPath(start_to_end).setLinearHeadingInterpolation(0.0, 0.0)
                .addPath(end_to_start).setLinearHeadingInterpolation(0.0, 0.785)
                .addPath(start_to_end).setLinearHeadingInterpolation(0.785, -0.125)
                .addPath(end_to_start).setLinearHeadingInterpolation(-0.125, 0.785)
                .addPath(start_to_end).setLinearHeadingInterpolation(0.785, 0.125)
                .addPath(end_to_start).setLinearHeadingInterpolation(0.125, 0.785);
        return builder.build();
    }
}
