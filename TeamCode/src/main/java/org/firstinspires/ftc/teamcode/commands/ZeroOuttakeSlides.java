package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.core.state.outtake.OuttakeState;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class ZeroOuttakeSlides extends CommandBase {
    private final AutonomousRobot robot;
    private Timer delay = new Timer();

    public ZeroOuttakeSlides(AutonomousRobot autonomousRobot) {
        robot = autonomousRobot;
        addRequirements(robot);
    }

    @Override
    public void initialize() {
        delay.resetTimer();
    }

    @Override
    public void execute() {
        robot.robot.drivetrain.motors.leftOuttakeSlide.set(-1.0);
        robot.robot.drivetrain.motors.rightOuttakeSlide.set(-1.0);
    }

    @Override
    public boolean isFinished() {
        if (delay.getElapsedTimeSeconds() > 0.3) {
            robot.robot.state.outtake.outtakeState = OuttakeState.DownClawOpen;
            robot.robot.drivetrain.motors.leftOuttakeSlide.set(0.0);
            robot.robot.drivetrain.motors.rightOuttakeSlide.set(0.0);
            return true;
        }
        return false;
    }
}
