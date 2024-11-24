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
        this.robot = new AutonomousRobot(telemetry, hardwareMap, follower);
        this.follower.setStartingPose(new Pose(0.0, 0.0, 0.0));
        this.chain = Paths.specimenAutoTest;
    }

    @Override
    public void start() {
        this.follower.update();
        this.follower.followPath(this.chain.getPath(0), true);
    }

    @Override
    public void loop() {
        // Drive to the specimen
        while (!robot.isHalfWayThere()) robot.update();
        // Start extending intake, but then keep driving and extending until at sample AND intake is extended
        robot.extendIntakeForSpecimen();
        while (!(robot.isIntakeExtended() && robot.atPathEnd())) robot.update();
        // Then grab the sample
        while (!robot.isIntakeDoneGrabbing()) robot.update();
        // Once sample is grabbed, we can drive over to the basket
        follower.followPath(this.chain.getPath(1), true);
        while (!robot.isTransferReady()) robot.update();
        // Make sure we are at the basket, with the outtake up, before dumping
        robot.raiseSlidesForSpecimenDump();
        while (!(robot.areSlidesReadyForSpecimenDump() && robot.atPathEnd() && robot.isTransferReady())) {
            robot.robot.telemetry.addData("atEnd", follower.atParametricEnd());
            robot.robot.telemetry.addData("isBusy", follower.isBusy());
            robot.robot.telemetry.addData("TV", follower.getCurrentTValue());
            robot.robot.telemetry.addData("isReady", robot.isTransferReady());
            robot.robot.telemetry.addData("velocity", follower.getVelocityMagnitude());
            robot.update();
        }
        // Perform the dump and wait for it to finish before moving on
        robot.specimenClip();
        while (!robot.isClipDone()) robot.update();
        while (robot.robot.state.outtake.outtakeState != OuttakeState.DownClawOpen) robot.update();
        terminateOpModeNow();
    }
}
