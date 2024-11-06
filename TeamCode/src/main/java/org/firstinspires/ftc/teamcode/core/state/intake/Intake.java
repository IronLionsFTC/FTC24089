package org.firstinspires.ftc.teamcode.core.state.intake;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class Intake {
    public IntakeState intakeState = IntakeState.Retracted;
    public Timer intakeLiftServoTimer = new Timer();
    public Timer timeUntilClamp = new Timer();

    public void set(IntakeState state) {
        intakeState = state;
    }
    public void toggle() {
        switch (intakeState) {
            case Folded:
                set(IntakeState.Retracted);
                break;
            case Retracted:
                set(IntakeState.Extended);
                break;
            case Extended:
                set(IntakeState.Collecting);
                break;
            case Collecting:
                set(IntakeState.Evaluating);
                break;
            case Evaluating:
                intakeLiftServoTimer.resetTimer();
                set(IntakeState.Depositing);
                break;
            case Depositing:
                set(IntakeState.Dropping);
                break;
            default:
                set(IntakeState.Retracted);
                break;
        }
    }
}