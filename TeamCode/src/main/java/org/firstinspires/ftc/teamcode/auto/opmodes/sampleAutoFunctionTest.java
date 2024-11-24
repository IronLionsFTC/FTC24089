package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.auto.paths.Paths;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

@Autonomous(name = "sampleAutoTest", group = "Testing")
public class sampleAutoFunctionTest extends OpMode {
    public PathChain chain;
    public Follower follower;
    public AutonomousRobot robot;
    public boolean readyToDrive = false;
    public int stage = 0;

    @Override
    public void init() {
        this.follower = new Follower(hardwareMap);
        this.robot = new AutonomousRobot(telemetry, hardwareMap, follower);
        this.follower.setStartingPose(new Pose(0.0, 0.0, 0.0));
        this.chain = Paths.harrison_test;
    }

    @Override
    public void start() {
        this.follower.update();
        this.follower.followPath(this.chain.getPath(this.stage));
    }

    @Override
    public void loop() {
        while (follower.getCurrentTValue() < 0.5) follower.update();
        robot.extendIntakeForSample();
        while (!robot.isIntakeExtended()) robot.update();
        while (!robot.isIntakeDoneGrabbing()) robot.update();
        follower.followPath(this.chain.getPath(1));
        while (!robot.isTransferReady()) robot.update();
        robot.raiseSlidesForSampleDump();
        while (!(robot.areSlidesReadyForSampleDump() && robot.isAtEndOfPath() && robot.isTransferReady())) robot.update();
        robot.performDump();
        while (!robot.isDumpDone()) robot.update();
        follower.followPath(this.chain.getPath(2));
        while (!robot.isAtEndOfPath()) robot.update();
        robot.extendIntakeForSample();
        while (!robot.isIntakeExtended()) robot.update();
        while (!robot.isIntakeDoneGrabbing()) robot.update();
        follower.followPath(this.chain.getPath(3));
        while (!robot.isTransferReady()) robot.update();
        robot.raiseSlidesForSampleDump();
        while (!(robot.areSlidesReadyForSampleDump() && robot.isAtEndOfPath() && robot.isTransferReady())) robot.update();
        robot.performDump();
        while (!robot.isDumpDone()) robot.update();
        follower.followPath(this.chain.getPath(4));
        while (!robot.isAtEndOfPath()) robot.update();
        robot.extendIntakeForSample();
        while (!robot.isIntakeExtended()) robot.update();
        while (!robot.isIntakeDoneGrabbing()) robot.update();
        follower.followPath(this.chain.getPath(5));
        while (!robot.isTransferReady()) robot.update();
        robot.raiseSlidesForSampleDump();
        while (!(robot.areSlidesReadyForSampleDump() && robot.isAtEndOfPath() && robot.isTransferReady())) robot.update();
        robot.performDump();
        while (!robot.isDumpDone()) robot.update();
        terminateOpModeNow();
    }
}
