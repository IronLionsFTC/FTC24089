package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.auto.constants.Points;
import org.firstinspires.ftc.teamcode.auto.paths.Paths;
import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.core.state.Team;
import org.firstinspires.ftc.teamcode.core.state.intake.IntakeState;
import org.firstinspires.ftc.teamcode.core.state.outtake.OuttakeState;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

@Autonomous(name = "Park Only Auto", group = "_MAIN_")
public class Park extends OpMode {
    public Follower follower;
    public Robot robot;

    @Override
    public void init() {
        this.follower = new Follower(hardwareMap);
        this.follower.setStartingPose(Points.slantStartPose);
        this.robot = new Robot(hardwareMap ,telemetry, gamepad1, gamepad2, Team.Blue);
        this.robot.state.intake.intakeState = IntakeState.Retracted;
        this.robot.state.outtake.outtakeState = OuttakeState.Down;
        telemetry = new MultipleTelemetry(telemetry);
    }

    @Override
    public void start() {
        follower.followPath(Paths.start_to_park);
    }

    @Override
    public void loop() {
        robot.drivetrain.servos.setPositions(robot.state.outtake.outtakeState, robot.state.intake.intakeState, robot.drivetrain.motors, robot.state.intake.clawYaw, 0.0);
        follower.update();
    }
}
