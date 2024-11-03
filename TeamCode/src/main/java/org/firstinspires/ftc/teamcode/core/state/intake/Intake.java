package org.firstinspires.ftc.teamcode.core.state.intake;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class Intake {
    public IntakeState intakeState = IntakeState.Retracted;
    public Timer intakeLiftServoTimer = new Timer();
    public Timer timeUntilClamp = new Timer();

    public void toggle() {
        if (intakeState == IntakeState.Folded) {
            intakeState = IntakeState.Retracted;
        } else if (intakeState == IntakeState.Retracted) {
            intakeState = IntakeState.Extended;
        } else if (intakeState == IntakeState.Extended) {
            intakeState = IntakeState.Collecting;
        } else if (intakeState == IntakeState.Collecting) {
            intakeState = IntakeState.Evaluating;
        } else if (intakeState == IntakeState.Evaluating) {
            intakeState = IntakeState.Depositing;
            intakeLiftServoTimer.resetTimer();
        } else if (intakeState == IntakeState.Depositing) {
            intakeState = IntakeState.Dropping;
        } else {
            intakeState = IntakeState.Retracted;
        }
    }
}