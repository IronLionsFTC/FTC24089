package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;

public class GrabGameObjectWithIntake extends CommandBase {
    private AutonomousRobot robot;

    public GrabGameObjectWithIntake(AutonomousRobot autonomousRobot) {
        robot = autonomousRobot;
        addRequirements(robot);
    }

    // No initialise because it waits BEFORE doing anything.

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return robot.isIntakeExtended();
    }
}
