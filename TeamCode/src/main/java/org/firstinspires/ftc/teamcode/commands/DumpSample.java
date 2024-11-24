package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;

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
    public boolean isFinished() {
        return robot.isDumpDone();
    }
}
