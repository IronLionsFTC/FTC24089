package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.auto.paths.Paths;
import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.core.state.Team;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous(name = "Main Auto", group = "_MAIN_")
public class MainAuto extends OpMode {
    public int autostate = 1;
    public Follower follower;
    public Robot robot;
    public Timer timer;

    @Override
    public void init() {
        this.robot = new Robot(hardwareMap, telemetry, Team.Blue);
        this.follower = new Follower(hardwareMap);
        this.follower.setStartingPose(new Pose(9.0, 41.0, Math.toRadians(-90)));
        this.timer = new Timer();
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
        this.robot.update_auto();
        this.follower.update();
        autoUpdate();
        telemetry.addData("Auto state:", autostate);
        telemetry.addData("PAUSED:", autostate < 0);
        telemetry.update();
    }


    public void setAutoState(int n) {
        this.autostate = n;
        timer.resetTimer();
    }
    public boolean atEnd() { return !(this.follower.isBusy()); }
    public boolean busy() { return this.follower.isBusy(); }
    public int get_next() { return Math.abs(this.autostate) + 1; }
    public void next() { this.autostate = get_next(); }

    public void autoUpdate() {
        switch (this.autostate) {
            // if `case` == 0 then auto is over
            case 0: return;
            // Start -> basket
            case 1:
                this.follower.followPath(Paths.start_to_basket);
                setAutoState(2);

            // Deposit starting sample
            case 2:
                if (busy()) break;

                // Just a state change, doesn't need any delays
                robot.prepareTransferForPreloadedSample();
                set_starting_outtake_state();
                setAutoState(3);

            // Position to get bottom sample from yellow spike mark
            case 3:
                if (!outtake()) break;

                this.follower.followPath(Paths.yellow_spike.BOTTOM);
                setAutoState(4);

            // Start intake
            case 4:
                if (busy()) break;

                set_starting_intake_state();
                setAutoState(-4);
            case -4:
                if (intake()) setAutoState(5);

            // Move forward onto sample
            case 5:
                this.follower.followPath(Paths.yellow_spike.BOTTOM_intake);
                setAutoState(6);

            // Return to outtake with the sample
            case 6:
                if (busy()) break;

                this.follower.followPath(Paths.yellow_spike.BOTTOM_return);

                set_starting_outtake_state();
                setAutoState(7);

            // Outtake the sample
            case 7:
                if (busy()) break;

                if (outtake()) setAutoState(8);

            // Same deal but for the middle spike mark
            // Position to intake middle spike mark sample
            case 8:
                this.follower.followPath(Paths.yellow_spike.MIDDLE);
                set_starting_intake_state();
                setAutoState(9);

            // Start intake
            case 9:
                if (busy()) break;

                if (intake()) setAutoState(10);

            // Intake the sample
            case 10:
                this.follower.followPath(Paths.yellow_spike.MIDDLE_intake);
                setAutoState(11);

            // Return to baskets to outtake the sample
            case 11:
                if (busy()) break;

                this.follower.followPath(Paths.yellow_spike.MIDDLE_return);
                set_starting_outtake_state();
                setAutoState(12);

            // Outtake the sample
            case 12:
                if (busy()) break;

                if (outtake()) setAutoState(13);

            // Go to park position, ready to intake in teleop
            case 13:
                this.follower.followPath(Paths.outtake_to_park);
                setAutoState(14);

            case 14:
                if (busy()) break;

                setAutoState(0);
        }
    }

    public int outtakestate = 1;
    public Path outtakeClearanceIn;
    public Path outtakeClearanceOut;
    public void set_starting_outtake_state() {
        this.outtakestate = 1;
    }
    public boolean outtake() {
        switch (outtakestate) {
            case 1:
                if (robot.tryTransfer()) outtakestate = 2;
            case 2:
                if (robot.transferCompleted()) outtakestate = 3;
            case 3:
                robot.extendOuttakeToTop();
                outtakestate = -3;
            case -3:
                if (robot.areSlidesReady()) outtakestate = 4;
            case 4:
                this.follower.followPath(Paths.outtakeClearanceIn);
                timer.resetTimer();
                outtakestate = 5;
            case 5:
                if (busy()) break;
                robot.dropSample();
                // Give the robot some time to drop the sample
                if (timer.getElapsedTimeSeconds() < 0.5) break;
                this.follower.followPath(Paths.outtakeClearanceOut);
                outtakestate = 6;
            case 6:
                if (busy()) break;

                robot.lowerSlides();
                outtakestate = -6;
            case -6:
                return robot.areSlidesDown();
        }
        return false;
    }

    public int intakestate = 1;
    public void set_starting_intake_state() {
        this.intakestate = 1;
    }
    public boolean intake() {
        switch (intakestate) {
            case 1:
                robot.extendIntakeFromFoldedPosition();
                intakestate = 2;
            case 2:
                if (!robot.isIntakeExtended()) break;

                timer.resetTimer();
                intakestate = 3;
            case 3:
                robot.foldDownIntakeAndStartCollecting();
                // Time for the intake to fold down
                return timer.getElapsedTimeSeconds() > 0.5;
        }
        return false;
    }
}










