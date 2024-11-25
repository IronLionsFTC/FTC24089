package org.firstinspires.ftc.teamcode.auto.paths.old;

import org.firstinspires.ftc.teamcode.auto.constants.Points;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;

public class StartToPark {
    public static PathChain path() {
        PathBuilder builder = new PathBuilder();

        builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                Points.slantStart,
                                Points.humanAreaPark
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90));

        return builder.build();
    }
}
