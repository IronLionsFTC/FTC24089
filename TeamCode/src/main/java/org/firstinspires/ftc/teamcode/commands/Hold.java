package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.core.state.intake.IntakeState;

public class Hold extends CommandBase {
    private final AutonomousRobot robot;

    public Hold(AutonomousRobot autonomousRobot) {
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
        return robot.intakeTimer.getElapsedTimeSeconds() > 0.3;
    }
}
