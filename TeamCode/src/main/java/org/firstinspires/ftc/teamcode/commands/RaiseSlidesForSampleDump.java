package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.core.params.RobotParameters;
import org.firstinspires.ftc.teamcode.core.state.outtake.OuttakeState;

public class RaiseSlidesForSampleDump extends CommandBase {
    private final AutonomousRobot robot;

    public RaiseSlidesForSampleDump(AutonomousRobot autonomousRobot) {
        robot = autonomousRobot;
        addRequirements(robot);
    }

    @Override
    public void initialize() {
        robot.raiseSlidesForSampleDump();
    }

    @Override
    public void execute() {
        if (robot.outtakeTimer.getElapsedTimeSeconds() > 0.3) {
            robot.robot.state.outtake.outtakeState = OuttakeState.UpWaitingToFlip;
        }
    }

    @Override
    public boolean isFinished() {
        return robot.robot.state.outtake.outtakeState == OuttakeState.UpWaitingToFlip && robot.robot.drivetrain.motors.outtakePosition() > RobotParameters.SlideBounds.outtakeUp - 200.0;
    }
}
