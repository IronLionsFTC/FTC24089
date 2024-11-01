package org.firstinspires.ftc.teamcode.auto.paths;

import org.firstinspires.ftc.teamcode.auto.constants.Points;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class StartToBasket {
    public static PathChain path() {
        PathBuilder builder = new PathBuilder();
        builder
                .addPath(
                        // Line 1
                        new BezierCurve(
                                Points.slantStart,
                                new Point(6.868, 70.671, Point.CARTESIAN),
                                new Point(33.009, 61.809, Point.CARTESIAN),
                                new Point(29.022, 104.788, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .setReversed(true)
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(29.022, 104.788, Point.CARTESIAN),
                                new Point(25.255, 120.738, Point.CARTESIAN),
                                Points.basketOuttake
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-45));
        return builder.build();
    }
}
