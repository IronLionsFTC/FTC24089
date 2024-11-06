package org.firstinspires.ftc.teamcode.core.state.outtake;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class Outtake {
    public OuttakeState outtakeState = OuttakeState.Down;
    public boolean retract = false;
    public Timer armRaiseTimer = new Timer();

    public void set(OuttakeState state) {
        outtakeState = state;
    }
    public void toggle() {
        switch (outtakeState) {
            case Folded:
                set(OuttakeState.Down);
            case Down:
                retract = false;
                set(OuttakeState.Waiting);
            case Waiting:
                set(OuttakeState.Up);
            case Up:
                armRaiseTimer.resetTimer();
                set(OuttakeState.Deposit);
            case Deposit:
            case PassthroughDeposit:
                retract = false;
                set(OuttakeState.Down);
            case Passthrough:
                set(OuttakeState.PassthroughDeposit);
        }
    }
}