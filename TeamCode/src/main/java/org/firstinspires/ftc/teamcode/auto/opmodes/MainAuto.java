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
    public int autostate = 0;
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
            // Start -> basket
            case 1:
                this.follower.followPath(Paths.start_to_basket);
                pause();
                break;
            case -1:
                setIfPathEnd(get_next());

            // Deposit starting sample
            case 2:
                // Deposit starting sample here
                pause();
            case -2:
                // If done depositing - check state
                next();

            // Position to get bottom sample from yellow spike mark
            case 3:
                this.follower.followPath(Paths.get_spike_yellow_BOTTOM);
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

            case 5:

        }
    }
}
