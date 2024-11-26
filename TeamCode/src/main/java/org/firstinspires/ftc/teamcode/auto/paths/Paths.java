package org.firstinspires.ftc.teamcode.auto.paths;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

public class Paths {
    public static PathChain onePlusThree = OnePlusThree.path();
    public static PathChain zeroPlusFour = ZeroPlusFour.path();

    public static PathChain fiveSpecimen_initial = FiveSpecimen.initial_dump();
    public static PathChain fiveSpecimen_pushes = FiveSpecimen.pushes();
    public static PathChain fiveSpecimen_intake(int s) { return FiveSpecimen.intake(s); }
    public static PathChain fiveSpecimen_outtake(int s) { return FiveSpecimen.outtake(s); }

    public static PathChain fourSpecimen = FourPlusZero.path();
}
