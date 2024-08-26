package org.firstinspires.ftc.teamcode.opmodes.testing;

import androidx.annotation.NonNull;

// Roadrunner
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Other
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.core.Robot;

@Config
@Autonomous(name = "TestAuto", group = "Autonomous")
public class TestAuto extends LinearOpMode {
    public Robot robot;
    public MecanumDrive drive;

    public void runOpMode() throws InterruptedException {
        // Prestart
        robot = new Robot(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        waitForStart();
        if (isStopRequested()) return;

        // Autonomous
        Actions.runBlocking(
                new SequentialAction(
                        // This first action does the same thing as the second
                        new Action() {
                            @Override
                            public boolean run(@NonNull TelemetryPacket packet) {
                                telemetry.addLine("test line");
                                return false;
                            }
                        },
                        // This is just a more concise way of doing things
                        (telemetryPacket) -> {
                            telemetry.addLine("test line 2");
                            return false;
                        },

                        // This is another way to build a trajectory
                        // It is not as preferable because building trajectories on the fly is slow
                        // Try to build actions beforehand if possible
                        drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                                .splineTo(new Vector2d(10, 10), Math.toRadians(0))
                                .splineTo(new Vector2d(0, 0), Math.toRadians(0))
                                .splineTo(new Vector2d(10, -10), Math.PI)
                                .splineTo(new Vector2d(-10, 0), Math.toRadians(0))
                                .build()
                )
        );
    }
}

