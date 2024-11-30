package org.firstinspires.ftc.teamcode.auto.paths;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

public class Paths {
    public static PathChain onePlusThree = OnePlusThree.path();
    public static PathChain zeroPlusFour = ZeroPlusFour.path();
    public static PathChain debug = Debug.path();

    public static PathChain fiveSpecimen_initial = FiveSpecimen.initial_dump();
    public static PathChain fiveSpecimen_goto_1 = FiveSpecimen.goto_first_spike();
    public static PathChain fiveSpecimen_give_1 = FiveSpecimen.give_first_spike();
    public static PathChain fiveSpecimen_goto_2 = FiveSpecimen.goto_second_spike();
    public static PathChain fiveSpecimen_give_2 = FiveSpecimen.give_second_spike();
    public static PathChain fiveSpecimen_prepare = FiveSpecimen.prepare_for_cycles();
    public static PathChain fiveSpecimen_intake(int s) { return FiveSpecimen.intake(s); }
    public static PathChain fiveSpecimen_outtake(int s) { return FiveSpecimen.outtake(s); }
    public static PathChain fiveSpecimen_driveOntoSpecimen = FiveSpecimen.driveOntoSpecimen();

    public static PathChain fourSpecimen_initial = FourPlusZero.initial_dump();
    public static PathChain fourSpecimen_goto_1 = FourPlusZero.goto_first_spike();
    public static PathChain fourSpecimen_give_1 = FourPlusZero.give_first_spike();
    public static PathChain fourSpecimen_goto_2 = FourPlusZero.goto_second_spike();
    public static PathChain fourSpecimen_give_2 = FourPlusZero.give_second_spike();
    public static PathChain fourSpecimen_prepare = FourPlusZero.prepare_for_cycles();
    public static PathChain fourSpecimen_intake(int s) { return FourPlusZero.intake(s); }
    public static PathChain fourSpecimen_outtake(int s) { return FourPlusZero.outtake(s); }
    public static PathChain fourSpecimen_driveOntoSpecimen = FourPlusZero.driveOntoSpecimen();
}
