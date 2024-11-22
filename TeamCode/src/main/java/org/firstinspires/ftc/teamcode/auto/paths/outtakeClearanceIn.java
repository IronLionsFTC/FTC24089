package org.firstinspires.ftc.teamcode.auto.paths;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.auto.constants.Points;

public class outtakeClearanceIn {
    public static PathChain path() {
        PathBuilder builder = new PathBuilder();

        builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                Points.basketOuttake,
                                Points.basketOuttakeCloser
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-45));

        return builder.build();
    }
}
