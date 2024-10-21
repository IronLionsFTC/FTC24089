package org.firstinspires.ftc.teamcode.auto.opmodes;

import org.firstinspires.ftc.teamcode.auto.constants.Points;
import org.firstinspires.ftc.teamcode.auto.constants.Poses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

// This will start in the corner for blue and go from there
// Start facing toward the baskets with the center on the 45 degree angled corner of the human player area
@Autonomous(name = "TestPedroAuto", group = "Testing")
public class TestPedroAuto extends OpMode {
    public Path TestCurveBlueToRedHumanPlayer = new Path(
            new BezierCurve(
                    Points.blueHumanPlayerAngledPoint, // Start point
                    new Point(130.75, 13.9, Point.CARTESIAN), // Control point
                    Points.redHumanPlayerAngledPoint // End point
            )
    );

    public Follower follower;


    @Override
    public void init() {
        this.follower = new Follower(hardwareMap);
    }

    @Override
    public void start() {
        follower.setStartingPose(Poses.blueHumanPlayerAngledPoint);
        follower.followPath(this.TestCurveBlueToRedHumanPlayer);
    }

    @Override
    public void loop() {
        this.follower.update();
    }
}
