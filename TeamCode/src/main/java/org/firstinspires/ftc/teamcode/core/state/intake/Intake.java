package org.firstinspires.ftc.teamcode.core.state.intake;

import org.firstinspires.ftc.teamcode.core.params.RobotParameters;

public class Intake {
    public IntakeState intakeState = IntakeState.Retracted;
    public double clawYaw = RobotParameters.ServoBounds.intakeYawZero;
    public void set(IntakeState state) {
        intakeState = state;
    }
    public void toggle() {
        switch (intakeState) {
            case Retracted:
                set(IntakeState.ExtendedClawUp);
                clawYaw = RobotParameters.ServoBounds.intakeYawZero;
                break;
            case ExtendedClawUp:
                set(IntakeState.ExtendedClawDown);
                clawYaw = RobotParameters.ServoBounds.intakeYawZero;
                break;
            case ExtendedClawDown:
                set(IntakeState.Grabbing);
                break;
            case Grabbing:
                set(IntakeState.Transfer);
                break;
            default:
                set(IntakeState.Retracted);
                break;
        }
    }
}