package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.core.state.Team;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

@Autonomous(name = "CV Auto Testing", group = "_MAIN_")
public class CVDebug extends OpMode {
    Follower follower = new Follower(hardwareMap);
    AutonomousRobot robot = new AutonomousRobot(telemetry, hardwareMap, follower, Team.Blue);
    @Override
    public void init() {
        return;
    }

    @Override
    public void start() {
        robot.extendIntakeForSample();
        return;
    }

    @Override
    public void loop() {
        LLResult analysis = robot.robot.computerVision.analyse();
        if (analysis != null) {
            robot.robot.computerVision.sample.update(robot.robot.computerVision.getSampleCornerPositions(analysis));
            robot.robot.computerVision.sample.getDirection();
            robot.clawPos = 0.3;
        }
        robot.update();
        return;
    }
}
