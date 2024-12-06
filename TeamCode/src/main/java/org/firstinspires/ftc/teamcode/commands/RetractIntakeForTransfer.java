package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.core.state.intake.IntakeState;

public class RetractIntakeForTransfer extends CommandBase {
    private final AutonomousRobot robot;

    public RetractIntakeForTransfer(AutonomousRobot autonomousRobot) {
        robot = autonomousRobot;
        addRequirements(robot);
    }

    @Override
    public void initialize() {
        robot.intakeTimer.resetTimer();
        robot.robot.drivetrain.motors.leftOuttakeSlide.set(-0.4);
        robot.robot.drivetrain.motors.rightOuttakeSlide.set(-0.4);
        robot.retractIntake();
    }

    @Override
    public boolean isFinished() {
        robot.disablePedro = false;
        if (robot.isTransferReady() && robot.intakeTimer.getElapsedTimeSeconds() > 0.9) {
            robot.robot.drivetrain.motors.leftOuttakeSlide.set(0);
            robot.robot.drivetrain.motors.rightOuttakeSlide.set(0);
            return true;
        }
        return false;
    }
}
