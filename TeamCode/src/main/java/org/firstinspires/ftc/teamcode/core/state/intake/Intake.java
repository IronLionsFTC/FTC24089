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
            intakeState = IntakeState.Depositing;
        } else {
            intakeState = IntakeState.Retracted;
        }
    }
}