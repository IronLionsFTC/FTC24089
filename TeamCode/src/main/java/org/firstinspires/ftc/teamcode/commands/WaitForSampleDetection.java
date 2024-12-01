package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.core.Vec2;
import org.firstinspires.ftc.teamcode.core.state.ComputerVision;
import org.firstinspires.ftc.teamcode.core.state.outtake.OuttakeState;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class WaitForSampleDetection extends CommandBase {
    private final AutonomousRobot robot;
    private double cv_rotation = 0.0;
    private Vec2 sample_position = new Vec2(0.0, 0.0);
    private int frames_of_valid_detection = 0;
    private double last_distance = 0.0;
    private boolean startedHoming = false;
    private Timer homingTimer = new Timer();

    public WaitForSampleDetection(AutonomousRobot autonomousRobot) {
        robot = autonomousRobot;
        addRequirements(robot);
    }

    @Override
    public void initialize() {
        robot.robot.computerVision.start();
        robot.disablePedro = true;
    }

    @Override
    public void execute() {
        sample_position = robot.robot.computerVision.getSamplePosition(robot.robot.computerVision.analyse());
        if (robot.robot.computerVision.sample.currentRotation != 0.0) cv_rotation = robot.robot.computerVision.sample.currentRotation;
        else frames_of_valid_detection = 0;
        if (sample_position != null) {
            LLResult analysis = robot.robot.computerVision.analyse();
            if (analysis != null) {
                if (!startedHoming) {
                    startedHoming = true;
                    homingTimer.resetTimer();
                }

                double rightPower = (sample_position.x * 0.7) / 30;
                double forwardPower = (sample_position.y * -0.7) / 25;

                if (forwardPower != 0.0) forwardPower -= 0.2;

                if (forwardPower != 0.0) forwardPower += (Math.abs(forwardPower) / forwardPower) * 0.15;
                if (rightPower != 0.0) rightPower += (Math.abs(rightPower) / rightPower) * 0.15;

                if (Math.abs(forwardPower) > 0.5) forwardPower = Math.abs(forwardPower) / forwardPower * 0.5;
                if (Math.abs(rightPower) > 0.5) rightPower = Math.abs(rightPower) / rightPower * 0.5;

                robot.robot.drivetrain.motors.powers.leftFront = (rightPower - forwardPower);
                robot.robot.drivetrain.motors.powers.rightFront = (-rightPower - forwardPower);
                robot.robot.drivetrain.motors.powers.leftBack = (-rightPower - forwardPower);
                robot.robot.drivetrain.motors.powers.rightBack = (rightPower - forwardPower);
                robot.robot.drivetrain.motors.setDrivePowers();
                frames_of_valid_detection += 1;
                robot.robot.computerVision.sample.update(robot.robot.computerVision.getSampleCornerPositions(analysis));
                robot.robot.computerVision.sample.getDirection(); // This DOES return rotation, but also caches it so can be treated as void
                last_distance = sample_position.magnitude;
            } else frames_of_valid_detection = 0;
        } else {
            frames_of_valid_detection = 0;
            robot.robot.drivetrain.motors.stopMotors();
        }
        if (frames_of_valid_detection == 0) sample_position = new Vec2(0.0,0.0);

        robot.robot.drivetrain.motors.stopMotors();
        robot.clawPos = 0.64 + cv_rotation * 0.5; // rotate slower and abuse margin of error of claw
    }

    @Override
    public boolean isFinished() {
        if (frames_of_valid_detection > 15 && last_distance < 7.0) {
            robot.robot.computerVision.stop();
            robot.robot.drivetrain.motors.stopMotors();
            return true;
        }
        return false;
    }
}
