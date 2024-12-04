package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.SyncdDevice;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.core.Vec2;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class LookForSampleForRaceCondition extends CommandBase {
    private final AutonomousRobot robot;
    private final Follower follower;
    private int frames_of_valid_detection = 0;
    private double x = 0.0;
    private double y = 0.0;
    private double a = 0.0;

    public LookForSampleForRaceCondition(AutonomousRobot autonomousRobot, Follower f) {
        robot = autonomousRobot;
        follower = f;
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
            x = sample_position.x;
            y = sample_position.y;
            LLResult analysis = robot.robot.computerVision.analyse();
            if (analysis != null) {
                robot.robot.computerVision.sample.update(robot.robot.computerVision.getSampleCornerPositions(analysis));
                double rot = robot.robot.computerVision.sample.getDirection() * -355;
                if (Math.abs(rot - robot.sampleR) > 15.0) {
                    frames_of_valid_detection = 0;
                }
                robot.setSampleXYR(x / 14, y / 14, rot);
                frames_of_valid_detection += 1;
                a = robot.robot.computerVision.getSampleArea(analysis);
                robot.logDouble("area", a);
            }
        } else {
            frames_of_valid_detection = 0;
        }
    }

    @Override
    public boolean isFinished() {
        if (frames_of_valid_detection > 10 && x != 0.0 && a < 25.0 && !(a < 8.0 && y < -6.0)) {
            follower.breakFollowing();
            return true;
        }
        return false;
    }
}
