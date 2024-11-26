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

    public static int dumpX = 660;

    private static final Point start = pointmm(0,0);
    private static final Point dump1 = pointmm(dumpX-20,150);
    private static final Point int1 = pointmm(550,-650);
    private static final Point hook1_1 = pointmm(800, -620);
    private static final Point hook1_2 = pointmm(800, -710);
    private static final Point push1 = pointmm(200, -950);
    private static final Point hook2_1 = pointmm(800, -770);
    private static final Point hook2_2 = pointmm(800, -890);
    private static final Point push2 = pointmm(200, -1100);
    private static final Point int2 = pointmm(650, -1100);

    private static final Point grab1 = pointmm(700, -500);
    private static final Point grab2 = pointmm(700, -780);
    private static final Point give = pointmm(450, -600);

    private static final Point humanPlayerSpecimenIntake_prep = pointmm(600, -330);
    private static final Point humanPlayerSpecimenIntake= pointmm(400, -530);
    private static final Point o1 = pointmm(dumpX,100);
    private static final Point o2 = pointmm(dumpX,50);
    private static final Point o3 = pointmm(dumpX+5,0);
    private static final Point o4 = pointmm(dumpX+5,-50);

    public static PathChain initial_dump() {
        PathBuilder builder = new PathBuilder();
        return builder
                // Preloaded specimen
                .addPath(
                        line(start, dump1)
                ).setConstantHeadingInterpolation(rad(180))
                .build();
    }

    public static PathChain goto_first_spike() {
        return new PathBuilder()
                .addPath(
                        line(dump1, grab1)
                ).setLinearHeadingInterpolation(rad(180), rad(-45))
                .build();
    }

    public static PathChain give_first_spike() {
        return new PathBuilder()
                .addPath(
                        line(grab1, give)
                ).setLinearHeadingInterpolation(rad(-45), rad(-135))
                .build();
    }

    public static PathChain goto_second_spike() {
        return new PathBuilder()
                .addPath(
                        line(give, grab2)
                ).setLinearHeadingInterpolation(rad(-135), rad(-45))
                .build();
    }

    public static PathChain give_second_spike() {
        return new PathBuilder()
                .addPath(
                        line(grab2, give)
                ).setLinearHeadingInterpolation(rad(-45), rad(-135))
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
