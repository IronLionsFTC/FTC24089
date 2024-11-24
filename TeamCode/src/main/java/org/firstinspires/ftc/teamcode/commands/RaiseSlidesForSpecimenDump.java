package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.core.params.RobotParameters;
import org.firstinspires.ftc.teamcode.core.state.outtake.OuttakeState;

public class RaiseSlidesForSpecimenDump extends CommandBase {
    private final AutonomousRobot robot;

    public RaiseSlidesForSpecimenDump(AutonomousRobot autonomousRobot) {
        robot = autonomousRobot;
        addRequirements(robot);
    }

    @Override
    public void initialize() {
        robot.raiseSlidesForSpecimenDump();
    }

    @Override
    public void execute() {
        if (robot.outtakeTimer.getElapsedTimeSeconds() > 0.3) {
            robot.robot.state.outtake.outtakeState = OuttakeState.UpWithSpecimenWaitingToFlip;
        }
        if (robot.outtakeTimer.getElapsedTimeSeconds() > 0.7) {
            robot.robot.state.outtake.outtakeState = OuttakeState.UpWithSpecimenFlipped;
        }
    }

    @Override
    public boolean isFinished() {
        return robot.outtakeTimer.getElapsedTimeSeconds() > 1.5;
    }
}
