package org.firstinspires.ftc.teamcode.auto.paths;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class FourPlusZero {
    public static PathChain path() {
        double x_1 = 0.0;
        double x_2 = -3.0;
        double x_3 = -6.0;
        double x_4 = -9.0;

        // 810 left, 420 back
        Point prepickup = new Point(-22.2, 31, 1);
        Point pickup = new Point(-17.2, 31, 1);
        double pickupAngle = 0.0;

        PathBuilder builder = new PathBuilder();
        builder
                // ------------------------------------------ PREPLACED
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(0.0, 0.0, Point.CARTESIAN),
                                new Point(-25.6, x_1, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(0)
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(-25.6, x_1, Point.CARTESIAN),
                                new Point(-25.6, 25.6, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(0)
                .addPath(
                        new BezierLine(
                                new Point(-25.6, 25.6, Point.CARTESIAN),
                                new Point(-48.4, 25.6, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(0)
                .addPath(
                        new BezierLine(
                                new Point(-48.4, 25.6, Point.CARTESIAN),
                                new Point(-48.4, 37.4, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(0)
                .addPath(
                        new BezierLine(
                                new Point(-48.4, 37.4, Point.CARTESIAN),
                                new Point(-3, 42.4, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(0)

                // --------------------------------- DRIVE TO EXTENSION POINT
                .addPath(
                    new BezierLine(
                            new Point(-3, 42.4, Point.CARTESIAN),
                            prepickup
                    )
                )
                .setConstantHeadingInterpolation(Math.toRadians(pickupAngle))
                .addPath(
                        new BezierLine(
                                prepickup,
                                pickup
                        )
                )
                .setConstantHeadingInterpolation(0.0)
                .addPath(
                    new BezierLine(
                            pickup,
                            new Point(-25.6, x_2, Point.CARTESIAN)
                    )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))

                // ------------------------------ DRIVE TO EXENSION POINT
                .addPath( new BezierLine(
                        new Point(-25.6, x_2, Point.CARTESIAN),
                        prepickup
                )
        )
                .setConstantHeadingInterpolation(0.0)
                .addPath(
                        new BezierLine(
                                prepickup,
                                pickup
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(pickupAngle))
                .addPath( new BezierLine(
                        pickup,
                        new Point(-25.6, x_3, Point.CARTESIAN)
                )
        )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath( new BezierLine(
                                new Point(-25.6, x_3, Point.CARTESIAN),
                                prepickup
                        )
                )
                .setConstantHeadingInterpolation(0.0)
                .addPath(
                        new BezierLine(
                                prepickup,
                                pickup
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(pickupAngle))
                .addPath( new BezierLine(
                                pickup,
                                new Point(-25.6, x_4, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0));
        return builder.build();
    }
}