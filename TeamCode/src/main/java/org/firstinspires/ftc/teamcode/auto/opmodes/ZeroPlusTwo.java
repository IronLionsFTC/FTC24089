package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.auto.paths.Paths;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

@Autonomous(name = "ZeroPlusTwo", group = "Testing")
public class ZeroPlusTwo extends OpMode {
    public PathChain chain;
    public Follower follower;
    public AutonomousRobot robot;
    public boolean readyToDrive = false;
    public int stage = 0;

    @Override
    public void init() {
        this.follower = new Follower(hardwareMap);
        this.robot = new AutonomousRobot(telemetry, hardwareMap, follower);
        this.follower.setStartingPose(new Pose(9.757, 84.983, 0.0));
        this.chain = Paths.zeroPlusTwo;
    }

    @Override
    public void start() {
        this.follower.update();
        this.follower.followPath(this.chain.getPath(this.stage));
    }

    @Override
    public void loop() {
        // Drive to the first pre-placed sample, start extending intake early
        while (follower.getCurrentTValue() < 0.5) {
            follower.update();
            robot.update();
        }
        // Start extending intake, but then keep driving and extending until at sample AND intake is extended
        robot.extendIntakeForSample();
        while (!(robot.isIntakeExtended() && follower.atParametricEnd())) {
            robot.update();
            follower.update();
        }
        // Then grab the sample
        while (!robot.isIntakeDoneGrabbing()) {
            robot.update();
            follower.update();
        }
        // Once sample is grabbed, we can drive over to the basket
        follower.followPath(this.chain.getPath(1));
        while (!robot.isTransferReady()) {
            robot.update();
            follower.update();
        }
        // Make sure we are at the basket, with the outtake up, before dumping
        robot.raiseSlidesForSampleDump();
        while (!(robot.areSlidesReadyForSampleDump() && follower.atParametricEnd() && robot.isTransferReady())) {
            robot.update();
            follower.update();
        }
        // Perform the dump and wait for it to finish before moving on
        robot.performDump();
        while (!robot.isDumpDone()) {
            robot.update();
            follower.update();
        }
        // Start moving towards the second sample, this is really close so immediately extend intake
        follower.followPath(this.chain.getPath(2));
        robot.extendIntakeForSample();
        while (!(follower.atParametricEnd() && robot.isIntakeExtended())) {
            robot.update();
            follower.update();
        }
        // Wait for intake to grab
        while (!robot.isIntakeDoneGrabbing()) {
            robot.update();
            follower.update();
        }
        // Head back to the basket while retracting intake
        follower.followPath(this.chain.getPath(3));
        while (!robot.isTransferReady()) {
            robot.update();
            follower.update();
        }
        // Raise the slides, but wait until the robot is ready before dumping
        robot.raiseSlidesForSampleDump();
        while (!(robot.areSlidesReadyForSampleDump() && follower.atParametricEnd() && robot.isTransferReady())) {
            robot.update();
            follower.update();
        }
        robot.performDump();
        while (!robot.isDumpDone()) {
            robot.update();
            follower.update();
        }
        // Now drive over to the basket and finish
        follower.followPath(this.chain.getPath(4));
        while (!follower.atParametricEnd()) {
            robot.update();
            follower.update();
        }
        terminateOpModeNow();
    }
}
