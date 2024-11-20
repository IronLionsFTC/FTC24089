package org.firstinspires.ftc.teamcode.core.state.outtake;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class Outtake {
    public OuttakeState outtakeState = OuttakeState.DownClawOpen;

    public void set(OuttakeState state) {
        outtakeState = state;
    }
    public void toggle() {
        switch (outtakeState) {
            case DownClawOpen:
                set(OuttakeState.DownClawShut);
                break;
            case DownClawShut:
                set(OuttakeState.UpWaitingToFlip);
                break;
            case UpWaitingToFlip:
                set(OuttakeState.UpFlipped);
                break;
            case UpFlipped:
                set(OuttakeState.UpClawOpen);
                break;
            case UpClawOpen:
                set(OuttakeState.UpWaitingToGoDown);
                break;
            case UpWaitingToGoDown:
                set(OuttakeState.DownClawOpen);
                break;

            case UpWithSpecimenWaitingToFlip:
                set(OuttakeState.UpWithSpecimenFlipped);
                break;
            case UpWithSpecimenFlipped:
                set(OuttakeState.UpWithSpecimenOnBar);
                break;
            case UpWithSpecimenOnBar:
                set(OuttakeState.UpWithSpecimentGoingDown);
                break;
            case UpWithSpecimentGoingDown:
                set(OuttakeState.DownClawOpen);
                break;

            default:
                break;
        }
    }
}