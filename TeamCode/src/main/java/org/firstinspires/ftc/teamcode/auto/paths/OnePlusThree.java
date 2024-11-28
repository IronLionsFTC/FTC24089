package org.firstinspires.ftc.teamcode.auto.paths;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.auto.paths.Points;

public class OnePlusThree {
    public static PathChain path() {
        PathBuilder builder = new PathBuilder();
        builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                Points.SampleStart.start,
                                Points.SampleStart.submersible
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        // Line 1
                        new BezierLine(
                                Points.SampleStart.submersible,
                                Points.SampleStart.sampleA
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-180))
                .addPath(
                        new BezierLine(
                                Points.SampleStart.sampleA,
                                Points.SampleStart.basket
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-235))
                .addPath(
                        new BezierLine(
                                Points.SampleStart.basket,
                                Points.SampleStart.sampleB
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-235), Math.toRadians(-180))
                .addPath(
                        new BezierLine(
                                Points.SampleStart.sampleB,
                                Points.SampleStart.basket
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-235))
                .addPath(
                        new BezierLine(
                                Points.SampleStart.basket,
                                Points.SampleStart.sampleC
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-235), Math.toRadians(-135))
                .addPath(
                        new BezierLine(
                                Points.SampleStart.sampleC,
                                Points.SampleStart.basket
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(-235))
                .addPath(
                        new BezierLine(
                                Points.SampleStart.basket,
                                Points.SampleStart.parkCorner
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-235), Math.toRadians(90))
                .addPath(
                        new BezierLine(
                                Points.SampleStart.parkCorner,
                                Points.SampleStart.park
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
        ;
        return builder.build();
    }
}