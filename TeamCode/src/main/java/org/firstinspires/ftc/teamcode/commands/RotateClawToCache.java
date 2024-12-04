package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;

public class RotateClawToCache extends CommandBase {
    private final AutonomousRobot robot;

    public RotateClawToCache(AutonomousRobot autonomousRobot) {
        robot = autonomousRobot;
        addRequirements(robot);
    }

    @Override
    public void initialize() {
        robot.clawPos = 0.64 - (robot.sampleR / 355);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
