package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.core.state.intake.IntakeState;

public class Release extends CommandBase {
    private final AutonomousRobot robot;

    public Release(AutonomousRobot autonomousRobot) {
        robot = autonomousRobot;
        addRequirements(robot);
    }

    @Override
    public void initialize() {
        robot.intakeTimer.resetTimer();
    }

    @Override
    public void execute() {
        robot.robot.state.intake.intakeState = IntakeState.ExtendedGrabbingOffWallClawOpen;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
