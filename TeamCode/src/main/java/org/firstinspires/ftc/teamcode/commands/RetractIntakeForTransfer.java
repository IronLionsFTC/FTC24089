package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.core.state.intake.IntakeState;

public class RetractIntakeForTransfer extends CommandBase {
    private final AutonomousRobot robot;

    public RetractIntakeForTransfer(AutonomousRobot autonomousRobot) {
        robot = autonomousRobot;
        addRequirements(robot);
    }

    @Override
    public void initialize() {
        robot.intakeTimer.resetTimer();
        robot.retractIntake();
    }

    @Override
    public boolean isFinished() {
        return robot.isTransferReady() && robot.intakeTimer.getElapsedTimeSeconds() > 1.5;
    }
}
