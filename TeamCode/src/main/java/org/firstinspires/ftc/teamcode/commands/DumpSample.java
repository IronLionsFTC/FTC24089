package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.core.state.outtake.OuttakeState;

public class DumpSample extends CommandBase {
    private final AutonomousRobot robot;

    public DumpSample(AutonomousRobot autonomousRobot) {
        robot = autonomousRobot;
        addRequirements(robot);
    }

    @Override
    public void initialize() {
        robot.performDump();
    }

    @Override
    public void execute() {
        if (robot.outtakeTimer.getElapsedTimeSeconds() > 0.5) {
            robot.robot.state.outtake.outtakeState = OuttakeState.UpClawOpen;
        }
        if (robot.outtakeTimer.getElapsedTimeSeconds() > 0.7) {
            robot.robot.state.outtake.outtakeState = OuttakeState.UpWaitingToGoDown;
        }
    }

    @Override
    public boolean isFinished() {
        if (robot.outtakeTimer.getElapsedTimeSeconds() > 1.2) {
            robot.robot.state.outtake.outtakeState = OuttakeState.DownClawOpen;
            return true;
        }
        return false;
    }
}
