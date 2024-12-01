package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.core.state.intake.IntakeState;

public class GrabGameObjectWithIntake extends CommandBase {
    private final AutonomousRobot robot;

    public GrabGameObjectWithIntake(AutonomousRobot autonomousRobot) {
        robot = autonomousRobot;
        addRequirements(robot);
    }

    @Override
    public void initialize() {
        robot.intakeTimer.resetTimer();
    }

    @Override
    public void execute() {
        robot.closeIntakeClaw();
    }

    @Override
    public boolean isFinished() {
        boolean sampleDone = (robot.intakeTimer.getElapsedTimeSeconds() > 0.3 && robot.robot.state.intake.intakeState == IntakeState.Grabbing);
        boolean specimenDone = (robot.intakeTimer.getElapsedTimeSeconds() > 0.6 && robot.robot.state.intake.intakeState == IntakeState.ExtendedGrabbingOffWallClawShut);
        if (sampleDone || specimenDone) {
            robot.disablePedro = false;
            return true;
        }
        return false;
    }
}
