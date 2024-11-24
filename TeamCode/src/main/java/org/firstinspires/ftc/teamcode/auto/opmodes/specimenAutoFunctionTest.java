package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.auto.paths.Paths;
import org.firstinspires.ftc.teamcode.core.state.outtake.OuttakeState;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

@Autonomous(name = "specimenAutoTest", group = "Testing")
public class specimenAutoFunctionTest extends OpMode {
    public PathChain chain;
    public Follower follower;
    public AutonomousRobot robot;

    @Override
    public void init() {
        this.follower = new Follower(hardwareMap);
        this.robot = new AutonomousRobot(telemetry, hardwareMap);
        this.follower.setStartingPose(new Pose(0.0, 0.0, 0.0));
        this.chain = Paths.specimenAutoTest;
    }

    @Override
    public void start() {
        this.follower.update();
        this.follower.followPath(this.chain.getPath(0));
    }

    @Override
    public void loop() {
        // Drive to the specimen
        while (follower.getCurrentTValue() < 0.5) {
            follower.update();
            robot.update();
        }
        // Start extending intake, but then keep driving and extending until at sample AND intake is extended
        robot.extendIntakeForSpecimen();
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
        robot.raiseSlidesForSpecimenDump();
        while (!(robot.areSlidesReadyForSpecimenDump() && follower.atParametricEnd() && robot.isTransferReady())) {
            robot.update();
            follower.update();
        }
        // Perform the dump and wait for it to finish before moving on
        robot.specimenClip();
        while (!robot.isClipDone()) {
            robot.update();
            follower.update();
        }
        while (robot.robot.state.outtake.outtakeState != OuttakeState.DownClawOpen) {
            robot.update();
            follower.update();
        }
        terminateOpModeNow();
    }
}
