package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.core.Vec2;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class SlightForwards extends CommandBase {
    private final AutonomousRobot robot;
    private Timer timer = new Timer();

    public SlightForwards(AutonomousRobot autonomousRobot) {
        robot = autonomousRobot;
        addRequirements(robot);
    }

    @Override
    public void initialize() {
        timer.resetTimer();
        robot.disablePedro = true;
    }

    @Override
    public void execute() {
        double rightPower = 0.0;
        double forwardPower = -0.5;

        robot.robot.drivetrain.motors.powers.leftFront = (rightPower - forwardPower);
        robot.robot.drivetrain.motors.powers.rightFront = (-rightPower - forwardPower);
        robot.robot.drivetrain.motors.powers.leftBack = (-rightPower - forwardPower);
        robot.robot.drivetrain.motors.powers.rightBack = (rightPower - forwardPower);
        robot.robot.drivetrain.motors.setDrivePowers();
    }

    @Override
    public boolean isFinished() {
        if (timer.getElapsedTimeSeconds() > 0.06) {
            robot.robot.drivetrain.motors.stopMotors();
            return true;
        }
        return false;
    }
}
