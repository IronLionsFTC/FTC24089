package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.auto.paths.Paths;
import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.core.state.Team;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;


import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.auto.constants.Points;
import org.firstinspires.ftc.teamcode.auto.paths.Paths;
import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.core.state.Team;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

@Autonomous(name = "PushThenPark (DODGE)", group = "_MAIN_")
public class PushThenParkWithDodge extends OpMode {
    public Follower follower;
    public Robot robot;

    @Override
    public void init() {
        this.follower = new Follower(hardwareMap);
        this.follower.setStartingPose(Points.basketTileFrontStartPose);
        this.robot = new Robot(hardwareMap, telemetry, Team.Blue);
        telemetry = new MultipleTelemetry(telemetry);
    }

    @Override
    public void start() {
        follower.followPath(Paths.push_then_park_with_dodge);
    }

    @Override
    public void loop() {
        follower.update();
    }
}
