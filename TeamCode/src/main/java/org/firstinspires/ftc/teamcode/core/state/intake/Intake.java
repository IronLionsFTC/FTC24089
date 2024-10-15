package org.firstinspires.ftc.teamcode.core.state.intake;

public class Intake {
    public IntakeState intakeState = IntakeState.Folded;

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
        } else if (intakeState == IntakeState.Depositing) {
            intakeState = IntakeState.Dropping;
        } else {
            intakeState = IntakeState.Retracted;
        }
    }
}