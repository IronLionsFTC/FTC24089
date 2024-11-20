package org.firstinspires.ftc.teamcode.core.state.intake;

import org.firstinspires.ftc.teamcode.core.params.RobotParameters;
import org.firstinspires.ftc.teamcode.core.state.ComputerVision;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class Intake {
    public IntakeState intakeState = IntakeState.Retracted;
    public Timer foldIntakeBeforeRetraction = new Timer();
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
                cv.sample.currentRotation = 0.0;
                set(IntakeState.ExtendedClawDown);
                clawYaw = RobotParameters.ServoBounds.intakeYawZero;
                break;
            case ExtendedClawDown:
                cv.stop();
                set(IntakeState.Grabbing);
                cv.sample.currentRotation = 0.0;
                break;
            case Grabbing:
                set(IntakeState.Transfer);
                foldIntakeBeforeRetraction.resetTimer();
                break;
            case ExtendedGrabbingOffWallClawOpen:
                set(IntakeState.ExtendedGrabbingOffWallClawShut);
                break;
            case ExtendedGrabbingOffWallClawShut:
                set(IntakeState.Transfer);
                break;
            case ExtendedClawShut:
                set(IntakeState.ExtendedClawOpen);
                break;
            default:
                set(IntakeState.Retracted);
                break;
        }
    }
}