package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.core.state.intake.Intake;
import org.firstinspires.ftc.teamcode.core.state.intake.IntakeState;
import org.firstinspires.ftc.teamcode.core.state.outtake.OuttakeState;

public class ResetOpmode extends CommandBase {
    private final AutonomousRobot robot;

    public ResetOpmode(AutonomousRobot autonomousRobot) {
        robot = autonomousRobot;
        addRequirements(robot);
    }

    @Override
    public void initialize() {
        robot.robot.state.intake.intakeState = IntakeState.Retracted;
        robot.robot.state.outtake.outtakeState = OuttakeState.DownClawOpen;
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
