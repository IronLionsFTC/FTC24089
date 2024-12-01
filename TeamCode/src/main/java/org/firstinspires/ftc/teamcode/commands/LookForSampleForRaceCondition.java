package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.SyncdDevice;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.core.Vec2;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class LookForSampleForRaceCondition extends CommandBase {
    private final AutonomousRobot robot;
    private int frames_of_valid_detection = 0;

    public LookForSampleForRaceCondition(AutonomousRobot autonomousRobot) {
        robot = autonomousRobot;
        addRequirements(robot);
    }

    @Override
    public void initialize() {
        robot.robot.computerVision.start();
    }

    @Override
    public void execute() {
        Vec2 sample_position = robot.robot.computerVision.getSamplePosition(robot.robot.computerVision.analyse());
        if (robot.robot.computerVision.sample.currentRotation == 0.0) frames_of_valid_detection = 0;
        if (sample_position != null) {
            LLResult analysis = robot.robot.computerVision.analyse();
            if (analysis != null) {
                robot.robot.computerVision.sample.update(robot.robot.computerVision.getSampleCornerPositions(analysis));
                robot.robot.computerVision.sample.getDirection(); // This DOES return rotation, but also caches it so can be treated as void
                frames_of_valid_detection += 1;
            }
        } else {
            frames_of_valid_detection = 0;
        }
    }

    @Override
    public boolean isFinished() {
        return frames_of_valid_detection > 15;
    }
}
