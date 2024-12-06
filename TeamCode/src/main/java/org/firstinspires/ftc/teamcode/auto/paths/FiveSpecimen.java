package org.firstinspires.ftc.teamcode.auto.paths;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Config
public class FiveSpecimen {
    private static Point pointmm(double x, double y) { return new Point(x/25.4,y/25.4,Point.CARTESIAN); }
    private static Point pointin(double x, double y) { return new Point(x,y,Point.CARTESIAN); }
    private static double rad(double deg) { return deg / 180 * Math.PI; }
    private static BezierLine line(Point start, Point end) { return new BezierLine(start, end); }

    public static int dumpX = 720;

    private static final Point start = pointmm(0,0);
    private static final Point int2 = pointmm(650, -400);

    private static final Point grab1 = pointmm(415, -605);
    private static final Point grab2 = pointmm(415, -840);
    private static final Point give = pointmm(400, -620);

    private static final Point humanPlayerSpecimenIntake_prep = pointmm(600, -400);
    private static final Point humanPlayerSpecimenIntake = pointmm(410, -580);

    private static final Point o0 = pointmm(dumpX,225);
    private static final Point o1 = pointmm(dumpX,150);
    private static final Point o2 = pointmm(dumpX,75);
    private static final Point o3 = pointmm(dumpX,0);
    private static final Point o4 = pointmm(dumpX,-75);

    public static PathChain initial_dump() {
        PathBuilder builder = new PathBuilder();
        return builder
                // Preloaded specimen
                .addPath(
                        line(start, o0)
                ).setConstantHeadingInterpolation(rad(180))
                .build();
    }

    public static PathChain goto_first_spike() {
        return new PathBuilder()
                .addPath(
                        line(o0, grab1)
                ).setLinearHeadingInterpolation(rad(180), rad(-45))
                .build();
    }

    public static PathChain give_first_spike() {
        return new PathBuilder()
                .addPath(
                        line(grab1, give)
                ).setConstantHeadingInterpolation(rad(-135))
                .build();
    }

    public static PathChain goto_second_spike() {
        return new PathBuilder()
                .addPath(
                        line(give, grab2)
                ).setConstantHeadingInterpolation(rad(-45))
                .build();
    }

    public static PathChain give_second_spike() {
        return new PathBuilder()
                .addPath(
                        line(grab2, give)
                ).setConstantHeadingInterpolation(rad(-135))
                .build();
    }

    public static PathChain prepare_for_cycles() {
        return new PathBuilder()
                .addPath(
                        line(give, int2)
                ).setConstantHeadingInterpolation(rad(-135))
                .build();
    }

    public static PathChain intake(int s) {
        Point current;

        switch (s) {
            case 1: current = int2; break;
            case 2: current = o1; break;
            case 3: current = o2; break;
            case 4: current = o3; break;
            default: return new PathChain();
        }

        return new PathBuilder()
                .addPath(
                        line(current, humanPlayerSpecimenIntake_prep)
                ).setLinearHeadingInterpolation(rad(180), rad(-135))
                .build();
    }

    public static PathChain outtake(int s) {
        Point target;

        switch (s) {
            case 1: target = o1; break;
            case 2: target = o2; break;
            case 3: target = o3; break;
            case 4: target = o4; break;
            default: return new PathChain();
        }

        return new PathBuilder()
                .addPath(
                        line(humanPlayerSpecimenIntake, target)
                ).setLinearHeadingInterpolation(rad(-135), rad(180))
                .build();
    }

    public static PathChain driveOntoSpecimen() {
        return new PathBuilder()
                .addPath(
                        line(humanPlayerSpecimenIntake_prep, humanPlayerSpecimenIntake)
                ).setConstantHeadingInterpolation(rad(-135))
                .build();
    }
}
