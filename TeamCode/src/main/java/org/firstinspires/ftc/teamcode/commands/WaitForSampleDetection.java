package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.core.Vec2;
import org.firstinspires.ftc.teamcode.core.state.ComputerVision;
import org.firstinspires.ftc.teamcode.core.state.outtake.OuttakeState;

public class WaitForSampleDetection extends CommandBase {
    private final AutonomousRobot robot;
    private double cv_rotation = 0.0;
    private Vec2 sample_position = new Vec2(0.0, 0.0);
    private int frames_of_valid_detection = 0;
    private double last_distance = 0.0;

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
                frames_of_valid_detection += 1;
                robot.robot.computerVision.sample.update(robot.robot.computerVision.getSampleCornerPositions(analysis));
                robot.robot.computerVision.sample.getDirection(); // This DOES return rotation, but also caches it so can be treated as void
                last_distance = sample_position.magnitude;
            } else frames_of_valid_detection = 0;
        } else frames_of_valid_detection = 0;
        if (frames_of_valid_detection == 0) sample_position = new Vec2(0.0,0.0);

        double rightPower = (sample_position.x * 1.5) / 30;
        double forwardPower = (sample_position.y * -1.5) / 25;

        robot.robot.drivetrain.motors.powers.leftFront = (rightPower - forwardPower);
        robot.robot.drivetrain.motors.powers.rightFront = (-rightPower - forwardPower);
        robot.robot.drivetrain.motors.powers.leftBack = (-rightPower - forwardPower);
        robot.robot.drivetrain.motors.powers.rightBack = (rightPower - forwardPower);
        robot.robot.drivetrain.motors.setDrivePowers();

        robot.clawPos = 0.64 + cv_rotation * 0.5; // rotate slower and abuse margin of error of claw
    }

    @Override
    public boolean isFinished() {
        if (frames_of_valid_detection > 35 && last_distance < 3.0) {
            robot.disablePedro = false;
            robot.robot.computerVision.stop();
            robot.robot.drivetrain.motors.stopMotors();
            robot.robot.drivetrain.motors.setDrivePowers();
            return true;
        }
        return false;
    }
}
