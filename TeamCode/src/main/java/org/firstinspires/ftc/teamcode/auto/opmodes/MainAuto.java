package org.firstinspires.ftc.teamcode.auto.opmodes;

import org.firstinspires.ftc.teamcode.auto.constants.Points;
import org.firstinspires.ftc.teamcode.auto.constants.Poses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.auto.paths.Paths;
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

    @Override
    public void init() {
        this.follower = new Follower(hardwareMap);
        this.follower.setStartingPose(new Pose(9.0, 41.0, Math.toRadians(-90)));
    }

    @Override
    public void start() {
        this.autostate = 1;
    }

    @Override
    public void loop() {
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
                nextIfPathEnd();

            // Deposit starting sample
            case 2:
                // Deposit starting sample here
                pause();
            case -2:
                // If done depositing - check state
                next();

            // Position to get bottom sample from yellow spike mark
            case 3:
                this.follower.followPath(Paths.yellow_spike.BOTTOM);
                pause();
            case -3:
                nextIfPathEnd();

            // Start intake
            case 4:
                // Start intake here
                pause();
            case -4:
                // MAY NOT BE NECESSARY
                // Check if ready to intake the sample
                next();

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
                // Outtake the sample here
                pause();
            case -7:
                // Ensure that outtake is complete and robot is ready for the next cycle
                next();

            // Same deal but for the middle spike mark

            // Position to intake middle spike mark sample
            case 8:
                this.follower.followPath(Paths.yellow_spike.MIDDLE);
                pause();
            case -8:
                nextIfPathEnd();

            // Start intake
            case 9:
                // start intake here
                pause();
            case -9:
                // MAY NOT BE NECESSARY
                // Check if ready to intake the sample
                next();

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
                // Outtake the sample here
                pause();
            case -12:
                // Ensure that the robot is ready to go
                next();

            // Go to park position, ready to intake in teleop
            case 13:
                this.follower.followPath(Paths.outtake_to_park);
                pause();
            case -13:
                setIfPathEnd(0); // End auto
        }
    }
}










