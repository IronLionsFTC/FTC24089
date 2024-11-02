package org.firstinspires.ftc.teamcode.auto.constants;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

public final class Points {
    public static final Point basketOuttake = new Point(20.15, 123.8, Point.CARTESIAN);
    public static final Point basketOuttakeCloser = new Point(17.150, 126.800, Point.CARTESIAN);
    public static final Point slantStart = new Point(9.0, 45.0, Point.CARTESIAN);
    public static final Point humanAreaPark = new Point(10.0, 11.0, Point.CARTESIAN);

    public static final Pose slantStartPose = new Pose(slantStart.getX(), slantStart.getY(), Math.PI/2);
    public static final Pose basketTileFrontStartPose = new Pose(9.0, 87.0, Math.PI/2);
}
