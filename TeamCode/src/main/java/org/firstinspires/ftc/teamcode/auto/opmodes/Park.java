package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.auto.paths.Paths;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

@Autonomous(name = "Park Only Auto", group = "_MAIN_")
public class Park extends OpMode {
    public Follower follower;

    @Override
    public void init() {
        this.follower = new Follower(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry);
    }

    @Override
    public void start() {
        follower.followPath(Paths.start_to_park);
    }

    @Override
    public void loop() {
        follower.update();
    }
}