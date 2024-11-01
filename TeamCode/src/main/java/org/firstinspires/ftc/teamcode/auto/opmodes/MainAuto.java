package org.firstinspires.ftc.teamcode.auto.opmodes;

import org.firstinspires.ftc.teamcode.auto.constants.Points;
import org.firstinspires.ftc.teamcode.auto.constants.Poses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.auto.paths.Paths;
import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.core.state.Team;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

@Autonomous(name = "Main Auto", group = "_MAIN_")
public class MainAuto extends OpMode {
    public int autostate = 1;
    public Follower follower;
    public Robot robot;

    @Override
    public void init() {
        this.robot = new Robot(hardwareMap, telemetry, Team.Blue);
        this.follower = new Follower(hardwareMap);
        this.follower.setStartingPose(new Pose(9.0, 41.0, Math.toRadians(-90)));
    }

    @Override
    public void start() {
        this.autostate = 1;
    }

    @Override
    public void loop() {
        this.robot.update_auto();
        this.follower.update();
        autoUpdate();
    }


    public void setAutoState(int n) { this.autostate = n; }
    public void setIfPathEnd(int n) {
        if (this.follower.atParametricEnd()) { this.autostate = n; }
    }
    public void pause() { this.autostate *= -1; }
    public int get_next() { return Math.abs(this.autostate) + 1; }
    public void next() { this.autostate = get_next(); }
    public void nextIfPathEnd() { setIfPathEnd(get_next()); }

    public void autoUpdate() {
        switch (this.autostate) {
            // if `case` == 0 then auto is over
            case 0:
                return;
            // Start -> basket
            case 1:
                this.follower.followPath(Paths.start_to_basket);
                pause();
            case -1:
                setIfPathEnd(2000);

            // Deposit starting sample
            case 2000:
                robot.prepareTransferForPreloadedSample();
                pause();
            case -2000:
                if (robot.tryTransfer()) {
                    setAutoState(-2001);
                }
            case -2001:
                if (robot.transferCompleted()) {
                    setAutoState(2002);
                }
            case 2002:
                robot.extendOuttakeToTop();
                pause();
            case -2002:
                if (robot.areSlidesReady()) {
                    setAutoState(2003);
                }
            case 2003:
                // TODO: adjust robot position toward basket if needed
                // Then move on to 2004
            case 2004:
                robot.dropSample();
                // TODO: make sure it is clear here and has moved back to standard position
                setAutoState(2005);
            case 2005:
                robot.lowerSlides();
                pause();
            case -2005:
                if (robot.areSlidesDown()) {
                    setAutoState(3);
                }


            // Position to get bottom sample from yellow spike mark
            case 3:
                this.follower.followPath(Paths.yellow_spike.BOTTOM);
                pause();
            case -3:
                nextIfPathEnd();

            // Start intake
            case 4:
                robot.extendIntakeFromFoldedPosition();
                pause();
            case -4:
                if (robot.isIntakeExtended()) {
                    setAutoState(4001);
                }
            case 4001:
                robot.foldDownIntakeAndStartCollecting();
                setAutoState(5);


            // Move forward onto sample
            case 5:
                this.follower.followPath(Paths.yellow_spike.BOTTOM_intake);
                pause();
            case -5:
                nextIfPathEnd();

            // May need to do some checking of the intake here

            // Return to outtake with the sample
            case 6:
                this.follower.followPath(Paths.yellow_spike.BOTTOM_return);
                pause();
            case -6:
                setIfPathEnd(7000);

            // Outtake the sample
            case 7000:
                robot.prepareTransferForPreloadedSample();
                pause();
            case -7000:
                if (robot.tryTransfer()) {
                    setAutoState(-7001);
                }
            case -7001:
                if (robot.transferCompleted()) {
                    setAutoState(7002);
                }
            case 7002:
                robot.extendOuttakeToTop();
                pause();
            case -7002:
                if (robot.areSlidesReady()) {
                    setAutoState(7003);
                }
            case 7003:
                // TODO: adjust robot position toward basket if needed
                // Then move on to 2004
            case 7004:
                robot.dropSample();
                // TODO: make sure it is clear here and has moved back to standard position
                setAutoState(7005);
            case 7005:
                robot.lowerSlides();
                pause();
            case -7005:
                if (robot.areSlidesDown()) {
                    setAutoState(8);
                }

            // Same deal but for the middle spike mark

            // Position to intake middle spike mark sample
            case 8:
                this.follower.followPath(Paths.yellow_spike.MIDDLE);
                pause();
            case -8:
                nextIfPathEnd();

            // Start intake
            case 9:
                robot.extendIntakeFromFoldedPosition();
                pause();
            case -9:
                if (robot.isIntakeExtended()) {
                    setAutoState(9001);
                }
            case 9001:
                robot.foldDownIntakeAndStartCollecting();
                setAutoState(10);

            // Intake the sample
            case 10:
                this.follower.followPath(Paths.yellow_spike.MIDDLE_intake);
                pause();
            case -10:
                nextIfPathEnd();

            // Return to baskets to outtake the sample
            case 11:
                this.follower.followPath(Paths.yellow_spike.MIDDLE_return);
                pause();
            case -11:
                setIfPathEnd(12000);

            // Outtake the sample
            case 12000:
                robot.prepareTransferForPreloadedSample();
                pause();
            case -12000:
                if (robot.tryTransfer()) {
                    setAutoState(-12001);
                }
            case -12001:
                if (robot.transferCompleted()) {
                    setAutoState(12002);
                }
            case 12002:
                robot.extendOuttakeToTop();
                pause();
            case -12002:
                if (robot.areSlidesReady()) {
                    setAutoState(12003);
                }
            case 12003:
                // TODO: adjust robot position toward basket if needed
                // Then move on to 2004
            case 12004:
                robot.dropSample();
                // TODO: make sure it is clear here and has moved back to standard position
                setAutoState(12005);
            case 12005:
                robot.lowerSlides();
                pause();
            case -12005:
                if (robot.areSlidesDown()) {
                    setAutoState(13);
                }

            // Go to park position, ready to intake in teleop
            case 13:
                this.follower.followPath(Paths.outtake_to_park);
                pause();
            case -13:
                setIfPathEnd(0); // End auto
        }
    }
}










