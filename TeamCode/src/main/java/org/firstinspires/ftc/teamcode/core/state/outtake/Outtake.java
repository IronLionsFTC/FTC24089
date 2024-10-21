package org.firstinspires.ftc.teamcode.core.state.outtake;

public class Outtake {
    public OuttakeState outtakeState = OuttakeState.Folded;

    public void toggle() {
        if (outtakeState == OuttakeState.Folded) {
            outtakeState = OuttakeState.Down;
        } else if (outtakeState == OuttakeState.Down) {
            outtakeState = OuttakeState.Up;
        } else if (outtakeState == OuttakeState.Up) {
            outtakeState = OuttakeState.Deposit;
        } else if (outtakeState == OuttakeState.Deposit || outtakeState == OuttakeState.PassthroughDeposit) {
            outtakeState = OuttakeState.Down;
        } else if (outtakeState == OuttakeState.Passthrough) {
            outtakeState = OuttakeState.PassthroughDeposit;
        }
    }
}
