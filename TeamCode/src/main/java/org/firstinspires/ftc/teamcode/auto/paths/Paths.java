package org.firstinspires.ftc.teamcode.auto.paths;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

public class Paths {
    public static PathChain start_to_basket = org.firstinspires.ftc.teamcode.auto.paths.StartToBasket.path();
    public static PathChain outtake_to_park = org.firstinspires.ftc.teamcode.auto.paths.OuttakeToPark.path();
    public static PathChain start_to_park = org.firstinspires.ftc.teamcode.auto.paths.StartToPark.path();
    public static PathChain push_then_park = org.firstinspires.ftc.teamcode.auto.paths.BasketStartPushToPark.path();
    public static PathChain push_then_park_with_dodge = org.firstinspires.ftc.teamcode.auto.paths.PushThenParkWithDodge.path();
    public static PathChain push_then_park_no_dodge = org.firstinspires.ftc.teamcode.auto.paths.PushThenParkNoDodge.path();
    public static PathChain sideways_start_to_park = org.firstinspires.ftc.teamcode.auto.paths.SidewaysStartToPark.path();
    public static PathChain FINALS = org.firstinspires.ftc.teamcode.auto.paths.FINALS.path();
    public static PathChain harrison_test = org.firstinspires.ftc.teamcode.auto.paths.harrison_test_for_auto_functions.path();
    public static PathChain outtakeClearanceIn = org.firstinspires.ftc.teamcode.auto.paths.outtakeClearanceIn.path();
    public static PathChain outtakeClearanceOut = org.firstinspires.ftc.teamcode.auto.paths.outtakeClearanceOut.path();

    public static class yellow_spike {
        public static PathChain BOTTOM = org.firstinspires.ftc.teamcode.auto.paths.yellow_spike.BOTTOM.path();
        public static PathChain BOTTOM_intake = org.firstinspires.ftc.teamcode.auto.paths.yellow_spike.BOTTOM_intake.path();
        public static PathChain BOTTOM_return = org.firstinspires.ftc.teamcode.auto.paths.yellow_spike.BOTTOM_return.path();
        public static PathChain MIDDLE = org.firstinspires.ftc.teamcode.auto.paths.yellow_spike.MIDDLE.path();
        public static PathChain MIDDLE_intake = org.firstinspires.ftc.teamcode.auto.paths.yellow_spike.MIDDLE_intake.path();
        public static PathChain MIDDLE_return = org.firstinspires.ftc.teamcode.auto.paths.yellow_spike.MIDDLE_return.path();
    }
}
