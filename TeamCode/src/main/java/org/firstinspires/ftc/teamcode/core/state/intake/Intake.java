package org.firstinspires.ftc.teamcode.core.state.intake;

import org.firstinspires.ftc.teamcode.core.params.RobotParameters;
import org.firstinspires.ftc.teamcode.core.state.ComputerVision;

public class Intake {
    public IntakeState intakeState = IntakeState.Retracted;
    public ComputerVision cv;
    public double clawYaw = RobotParameters.ServoBounds.intakeYawZero;
    public void set(IntakeState state) {
        intakeState = state;
    }
    public Intake(ComputerVision computerVision) {
        cv = computerVision;
    }
    public void toggle() {
        switch (intakeState) {
            case Retracted:
                set(IntakeState.ExtendedClawUp);
                clawYaw = RobotParameters.ServoBounds.intakeYawZero;
                break;
            case ExtendedClawUp:
                cv.start();
                set(IntakeState.ExtendedClawDown);
                clawYaw = RobotParameters.ServoBounds.intakeYawZero;
                break;
            case ExtendedClawDown:
                cv.stop();
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