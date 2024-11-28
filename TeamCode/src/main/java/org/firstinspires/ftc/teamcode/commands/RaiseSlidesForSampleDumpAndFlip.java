package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.core.params.RobotParameters;
import org.firstinspires.ftc.teamcode.core.state.outtake.OuttakeState;

public class RaiseSlidesForSampleDumpAndFlip extends CommandBase {
    private final AutonomousRobot robot;

    public RaiseSlidesForSampleDumpAndFlip(AutonomousRobot autonomousRobot) {
        robot = autonomousRobot;
        addRequirements(robot);
    }

    @Override
    public void initialize() {
        robot.robot.state.outtake.outtakeState = OuttakeState.DownClawShut;
        robot.outtakeTimer.resetTimer();
    }

    @Override
    public void execute() {
        if (robot.outtakeTimer.getElapsedTimeSeconds() > 0.3 && robot.robot.state.outtake.outtakeState == OuttakeState.DownClawShut) {
            robot.robot.state.outtake.outtakeState = OuttakeState.UpWaitingToFlip;
        }
        if (robot.outtakeTimer.getElapsedTimeSeconds() > 1.8 && robot.robot.state.outtake.outtakeState == OuttakeState.UpWaitingToFlip) {
            robot.robot.state.outtake.outtakeState = OuttakeState.UpFlipped;
        }
    }

    @Override
    public boolean isFinished() {
        if (robot.robot.state.outtake.outtakeState == OuttakeState.UpFlipped && robot.robot.drivetrain.motors.outtakePosition() > RobotParameters.SlideBounds.outtakeUp - 250.0 && robot.outtakeTimer.getElapsedTimeSeconds() > 3.0) {
            return true;
        }
        return false;
    }
}
