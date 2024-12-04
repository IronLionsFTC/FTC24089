package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.core.Vec2;

public class FindSampleWithCV extends CommandBase {
    private final AutonomousRobot robot;
    private int frames_of_valid_detection = 0;
    private double cv_rotation = 0.0;

    public FindSampleWithCV(AutonomousRobot autonomousRobot) {
        robot = autonomousRobot;
        addRequirements(robot);
    }

    @Override
    public void initialize() {
        robot.robot.computerVision.start();
    }

    @Override
    public boolean isFinished() {
        Vec2 sample_position = robot.robot.computerVision.getSamplePosition(robot.robot.computerVision.analyse());
        if (robot.robot.computerVision.sample.currentRotation == 0.0) frames_of_valid_detection = 0;
        if (sample_position != null) {
            LLResult analysis = robot.robot.computerVision.analyse();
            if (analysis != null) {
                robot.robot.computerVision.sample.update(robot.robot.computerVision.getSampleCornerPositions(analysis));
                cv_rotation = robot.robot.computerVision.sample.getDirection(); // This DOES return rotation, but also caches it so can be treated as void
                frames_of_valid_detection += 1;
                robot.sampleX = (sample_position.x / 24.0) * (7.5 / 2.54);
                robot.sampleY = (sample_position.y / 20.0) * (6.5 / 2.54);
                robot.sampleR = cv_rotation;
            } else {
                frames_of_valid_detection = 0;
            }
        } else {
            frames_of_valid_detection = 0;
        }
        if (frames_of_valid_detection > 15 && robot.sampleX != 0.0) {
            robot.robot.computerVision.stop();
            return true;
        }
        return false;
    }
}
