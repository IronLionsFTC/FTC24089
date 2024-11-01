package org.firstinspires.ftc.teamcode.core.state.outtake;

public class Outtake {
    public OuttakeState outtakeState = OuttakeState.Down;
    public boolean retract = false;

    public void toggle() {
        if (outtakeState == OuttakeState.Folded) {
            outtakeState = OuttakeState.Down;
        } else if (outtakeState == OuttakeState.Down) {
            retract = false;
            outtakeState = OuttakeState.Waiting;
        } else if (outtakeState == OuttakeState.Waiting) {
            outtakeState = OuttakeState.Up;
        } else if (outtakeState == OuttakeState.Up) {
            outtakeState = OuttakeState.Deposit;
        } else if (outtakeState == OuttakeState.Deposit || outtakeState == OuttakeState.PassthroughDeposit) {
            retract = false;
            outtakeState = OuttakeState.Down;
        } else if (outtakeState == OuttakeState.Passthrough) {
            outtakeState = OuttakeState.PassthroughDeposit;
        }
    }
}