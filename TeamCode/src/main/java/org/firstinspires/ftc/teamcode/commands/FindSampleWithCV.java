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
        if (robot.robot.computerVision.sample.currentRotation != 0.0) cv_rotation = robot.robot.computerVision.sample.currentRotation;
        if (sample_position != null) {
            LLResult analysis = robot.robot.computerVision.analyse();
            if (analysis != null) {
                robot.robot.computerVision.sample.update(robot.robot.computerVision.getSampleCornerPositions(analysis));
                robot.robot.computerVision.sample.getDirection(); // This DOES return rotation, but also caches it so can be treated as void
                frames_of_valid_detection += 1;
                robot.sampleX = ((sample_position.x + 30.0) / 60.0) * (15.0 / 2.54);
                robot.sampleY = ((sample_position.y + 25.0) / 50.0) * (11.0 / 2.54);
                robot.sampleR = cv_rotation;
                return true;
            }
        } else {
            frames_of_valid_detection = 0;
        }
        return false;
    }
}
