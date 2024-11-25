package org.firstinspires.ftc.teamcode.auto.paths.old.yellow_spike;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import org.firstinspires.ftc.teamcode.auto.constants.Points;

public class MIDDLE_intake {
    public static PathChain path() {
        PathBuilder builder = new PathBuilder();

        builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(22.0, 132.0, Point.CARTESIAN),
                                new Point(30.0, 132.0, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(0);

        return builder.build();
    }
}
