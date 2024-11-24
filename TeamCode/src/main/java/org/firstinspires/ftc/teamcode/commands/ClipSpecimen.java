package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.core.state.outtake.OuttakeState;

public class ClipSpecimen extends CommandBase {
    private final AutonomousRobot robot;

    public ClipSpecimen(AutonomousRobot autonomousRobot) {
        robot = autonomousRobot;
        addRequirements(robot);
    }

    @Override
    public void initialize() {
        robot.specimenClip();
    }

    @Override
    public boolean isFinished() {
        if (robot.outtakeTimer.getElapsedTimeSeconds() > 3.0 || robot.robot.drivetrain.motors.outtakePosition() > AutonomousRobot.AutonomousTune.outtakeHeightToCancel) {
            robot.robot.state.outtake.outtakeState = OuttakeState.UpWithSpecimentGoingDown; // Set state here to be idiot proof [will always stop the slides]
            robot.outtakeTimer.resetTimer();
            return true;
        }
        return false;
    }
}
