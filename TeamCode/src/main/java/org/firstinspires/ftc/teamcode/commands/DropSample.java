package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.core.state.outtake.OuttakeState;

public class DropSample extends CommandBase {
    private final AutonomousRobot robot;

    public DropSample(AutonomousRobot autonomousRobot) {
        robot = autonomousRobot;
        addRequirements(robot);
    }

    @Override
    public void initialize() {
        robot.robot.state.outtake.outtakeState = OuttakeState.UpClawOpen;
        robot.outtakeTimer.resetTimer();
    }

    @Override
    public boolean isFinished() {
        if (robot.outtakeTimer.getElapsedTimeSeconds() > 0.6) {
            robot.robot.state.outtake.outtakeState = OuttakeState.UpWaitingToGoDown;
            return true;
        }
        return false;
    }
}
