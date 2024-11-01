package org.firstinspires.ftc.teamcode.auto.opmodes;

import org.firstinspires.ftc.teamcode.auto.constants.Points;
import org.firstinspires.ftc.teamcode.auto.constants.Poses;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
        telemetry = new MultipleTelemetry(telemetry);

        telemetry.addLine("READY");
        telemetry.update();
    }

    @Override
    public void start() {
        this.autostate = 1;
    }

    @Override
    public void loop() {
//        this.robot.update_auto();
        this.follower.update();
        autoUpdate();
        this.follower.update();
    }


    public void setAutoState(int n) { this.autostate = n; }
    public void setIfPathEnd(int n) {
        if (this.follower.isBusy()) { this.autostate = n; }
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
                nextIfPathEnd();

            // Deposit starting sample
            case 2:
                robot.prepareTransferForPreloadedSample();
                start_outtake();
                pause();
            case -2:
                if (outtake()) {
                    next();
                }


            // Position to get bottom sample from yellow spike mark
            case 3:
                this.follower.followPath(Paths.yellow_spike.BOTTOM);
                pause();
            case -3:
                nextIfPathEnd();

            // Start intake
            case 4:
                start_intake();
                pause();
            case -4:
                if (intake()) {
                    next();
                }

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
                nextIfPathEnd();

            // Outtake the sample
            case 7:
                start_outtake();
                pause();
            case -7:
                if (outtake()) {
                    next();
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
                start_intake();
                pause();
            case -9:
                if (intake()) {
                    next();
                }

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
                nextIfPathEnd();

            // Outtake the sample
            case 12:
                start_outtake();
                pause();
            case -12:
                if (outtake()) {
                    next();
                }

            // Go to park position, ready to intake in teleop
            case 13:
                this.follower.followPath(Paths.outtake_to_park);
                pause();
            case -13:
                setIfPathEnd(0); // End auto
        }
        telemetry.addData("Auto state:", autostate);
        telemetry.update();
    }

    public int outtakestate = 1;
    public Path outtakeClearanceIn;
    public Path outtakeClearanceOut;
    public void start_outtake() {
        this.outtakestate = 1;
    }
    public boolean outtake() {
        switch (outtakestate) {
            // If done
            case 0: return true;
            // Outtake the sample
            case 1:
                outtakestate = -1;
            case -1:
                if (robot.tryTransfer()) {
                    outtakestate = -1002;
                }
            case -1002:
                if (robot.transferCompleted()) {
                    outtakestate = 2;
                }
            case 2:
                robot.extendOuttakeToTop();
                outtakestate = -2;
            case -2:
                if (robot.areSlidesReady()) {
                    outtakestate = 3;
                }
            case 3:
                this.follower.followPath(Paths.outtakeClearanceIn);
                outtakestate = -3;
            case -3:
                if (this.follower.isBusy()) {
                    outtakestate = 4;
                }
            case 4:
                robot.dropSample();
                this.follower.followPath(Paths.outtakeClearanceOut);
                outtakestate = -4;
            case -4:
                if (this.follower.isBusy()) {
                    outtakestate = 5;
                }

            case 5:
                robot.lowerSlides();
                outtakestate = -5;
            case -5:
                if (robot.areSlidesDown()) {
                    outtakestate = 0;
                }
        }
        return false;
    }

    public int intakestate = 1;
    public void start_intake() {
        this.intakestate = 1;
    }
    public boolean intake() {
        switch (intakestate) {
            case 0: return true;

            case 1:
                robot.extendIntakeFromFoldedPosition();
                intakestate = -1;
            case -1:
                if (robot.isIntakeExtended()) {
                    intakestate = 2;
                }
            case 2:
                robot.foldDownIntakeAndStartCollecting();
                return true;
        }
        return false;
    }
}










