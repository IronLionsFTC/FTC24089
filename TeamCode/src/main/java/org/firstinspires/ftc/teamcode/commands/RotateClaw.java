package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.core.state.outtake.OuttakeState;

public class RotateClaw extends CommandBase {
    private final AutonomousRobot robot;
    private double degrees;

    public RotateClaw(AutonomousRobot autonomousRobot, double d) {
        robot = autonomousRobot;
        addRequirements(robot);
        degrees = d;
    }

    @Override
    public void initialize() {
        robot.clawPos = (degrees / 355);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
