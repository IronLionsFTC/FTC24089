package org.firstinspires.ftc.teamcode.auto.paths;

import android.provider.Telephony;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.auto.paths.Points.SampleStart;

import java.net.BindException;

public class ZeroPlusFour {
    public static PathChain path() {
        PathBuilder builder = new PathBuilder();
        builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                SampleStart.start,
                                SampleStart.closeBasket
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-235))
                .addPath(
                        // Line 1
                        new BezierLine(
                                SampleStart.closeBasket,
                                SampleStart.sampleA
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-235), Math.toRadians(-180))
                .addPath(
                        new BezierLine(
                                SampleStart.sampleA,
                                SampleStart.basket
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-235))
                .addPath(
                        new BezierLine(
                                SampleStart.basket,
                                SampleStart.sampleB
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-235), Math.toRadians(-180))
                .addPath(
                        new BezierLine(
                                SampleStart.sampleB,
                                SampleStart.basket
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-235))
                .addPath(
                        new BezierLine(
                                SampleStart.basket,
                                SampleStart.sampleC
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-235), Math.toRadians(-135))
                .addPath(
                        new BezierLine(
                                SampleStart.sampleC,
                                SampleStart.basket
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(-235))
                .addPath(
                        new BezierCurve(
                                SampleStart.basket,
                                SampleStart.controlPoint,
                                SampleStart.park
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(
                                SampleStart.park,
                                SampleStart.strafeEnd
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(90))
                .addPath(
                        new BezierLine(
                                SampleStart.strafeEnd,
                                SampleStart.park
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(90))
                .addPath(
                        new BezierCurve(
                                SampleStart.park,
                                SampleStart.controlPoint,
                                new Point(-12, -28, Point.CARTESIAN),
                                SampleStart.basket
                        )
                ).setTangentHeadingInterpolation()
                .setReversed(true)
        ;
        return builder.build();
    }
}