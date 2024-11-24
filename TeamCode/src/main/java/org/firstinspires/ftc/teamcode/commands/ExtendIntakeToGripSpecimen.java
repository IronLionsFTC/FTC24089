package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;

public class ExtendIntakeToGripSpecimen extends CommandBase {
    private AutonomousRobot robot;

    public ExtendIntakeToGripSpecimen(AutonomousRobot autonomousRobot) {
        robot = autonomousRobot;
        addRequirements(robot);
    }

    @Override
    public void initialize() {
        robot.extendIntakeForSpecimen();
    }

    @Override
    public boolean isFinished() {
        return robot.isIntakeExtended();
    }
}
