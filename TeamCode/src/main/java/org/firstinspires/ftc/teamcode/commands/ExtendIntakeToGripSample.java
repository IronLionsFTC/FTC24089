package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;

public class ExtendIntakeToGripSample extends CommandBase {
    private final AutonomousRobot robot;

    public ExtendIntakeToGripSample(AutonomousRobot autonomousRobot) {
        robot = autonomousRobot;
        addRequirements(robot);
    }

    @Override
    public void initialize() {
        robot.extendIntakeForSample();
    }

    @Override
    public boolean isFinished() {
        return robot.isIntakeExtended();
    }
}
