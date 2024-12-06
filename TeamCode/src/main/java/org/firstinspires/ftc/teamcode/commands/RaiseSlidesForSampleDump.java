package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.core.params.RobotParameters;
import org.firstinspires.ftc.teamcode.core.state.intake.IntakeState;
import org.firstinspires.ftc.teamcode.core.state.outtake.OuttakeState;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

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
        if (robot.outtakeTimer.getElapsedTimeSeconds() > 0.2)  {
            robot.robot.state.intake.intakeState = IntakeState.Retracted;
        }
        if (robot.outtakeTimer.getElapsedTimeSeconds() > 0.3 && robot.robot.state.outtake.outtakeState == OuttakeState.DownClawShut) {
            robot.robot.state.outtake.outtakeState = OuttakeState.UpWaitingToFlip;
        }
    }

    @Override
    public boolean isFinished() {
        if (robot.robot.state.outtake.outtakeState == OuttakeState.UpWaitingToFlip && robot.robot.drivetrain.motors.outtakePosition() > RobotParameters.SlideBounds.outtakeUp - 250.0) {
            return true;
        }
        return false;
    }
}
