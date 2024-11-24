package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.core.state.intake.IntakeState;

public class GrabGameObjectWithIntake extends CommandBase {
    private AutonomousRobot robot;

    public GrabGameObjectWithIntake(AutonomousRobot autonomousRobot) {
        robot = autonomousRobot;
        addRequirements(robot);
    }

    // No initialise because it waits BEFORE doing anything.

    @Override
    public void execute() {
        if (robot.intakeTimer.getElapsedTimeSeconds() > 0.6 && robot.robot.state.intake.intakeState == IntakeState.ExtendedClawDown) robot.closeIntakeClaw();
        if (robot.intakeTimer.getElapsedTimeSeconds() > 1.0 && robot.robot.state.intake.intakeState == IntakeState.ExtendedGrabbingOffWallClawOpen) robot.closeIntakeClaw();
        if (robot.intakeTimer.getElapsedTimeSeconds() > 1.0 && robot.robot.state.intake.intakeState == IntakeState.Grabbing) {
            if (robot.robot.state.intake.foldIntakeBeforeRetraction.getElapsedTimeSeconds() > 2.6) {
                robot.robot.state.intake.foldIntakeBeforeRetraction.resetTimer();
            }
            robot.robot.state.intake.intakeState = IntakeState.Transfer;
        }
        if (robot.intakeTimer.getElapsedTimeSeconds() > 2.0 && robot.robot.state.intake.intakeState == IntakeState.ExtendedGrabbingOffWallClawShut) {
            if (robot.robot.state.intake.foldIntakeBeforeRetraction.getElapsedTimeSeconds() > 2.6) {
                robot.robot.state.intake.foldIntakeBeforeRetraction.resetTimer();
            }
            robot.robot.state.intake.intakeState = IntakeState.Transfer;
        }
    }

    @Override
    public boolean isFinished() {
        boolean sampleDone = (robot.intakeTimer.getElapsedTimeSeconds() > 1.0 && robot.robot.state.intake.intakeState == IntakeState.Grabbing);
        boolean specimenDone = (robot.intakeTimer.getElapsedTimeSeconds() > 2.0 && robot.robot.state.intake.intakeState == IntakeState.ExtendedGrabbingOffWallClawShut);
        if (sampleDone || specimenDone) {
            if (robot.robot.state.intake.foldIntakeBeforeRetraction.getElapsedTimeSeconds() > 2.6) {
                robot.robot.state.intake.foldIntakeBeforeRetraction.resetTimer();
            }
            robot.robot.state.intake.intakeState = IntakeState.Transfer;
            return true;
        }
        return false;
    }
}
