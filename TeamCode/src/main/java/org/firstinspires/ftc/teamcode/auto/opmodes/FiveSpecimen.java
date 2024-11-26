package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.auto.paths.Paths;
import org.firstinspires.ftc.teamcode.commands.Commands;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

@Config
@Autonomous(name = "5 Specimen", group = "MAIN")
public class FiveSpecimen extends CommandOpMode {
    public PathChain chain;
    public Follower follower;
    public AutonomousRobot robot;

    private static double movementSpeedToIntake = 0.5;
    public static int outtakePathDelay = 500;
    public static int slideRaiseDelay = 1000;

    private Command dump(int s) {
        return Commands.fastPath(follower, Paths.fiveSpecimen_intake(s)).alongWith(
                Commands.ExtendIntakeToGripSpecimen(robot)
        ).andThen(
                Commands.followPath(follower, Paths.fiveSpecimen_driveOntoSpecimen).setSpeed(movementSpeedToIntake),
                Commands.RetractIntakeForTransfer(robot),
                Commands.GrabGameObjectWithIntake(robot).alongWith(
                        Commands.sleep(outtakePathDelay),
                        Commands.fastPath(follower, Paths.fiveSpecimen_outtake(s)).alongWith(
                                Commands.sleep(slideRaiseDelay),
                                Commands.RaiseSlidesForSpecimenDump(robot)
                        )
                ),
                Commands.ClipSpecimen(robot)
        );
    }

    @Override
    public void initialize() {
        this.follower = new Follower(hardwareMap);
        this.robot = new AutonomousRobot(telemetry, hardwareMap, follower);
        this.follower.setStartingPose(new Pose(0, 0, Math.PI));


        schedule(
                new RunCommand(robot::update),
                new SequentialCommandGroup(
                        Commands.sleepUntil(this::opModeIsActive),

                        // Dump preloaded specimen
                        Commands.fastPath(follower, Paths.fiveSpecimen_initial).alongWith(
                                Commands.RaiseSlidesForSpecimenDump(robot)
                        ),
                        Commands.ClipSpecimen(robot),

                        // Push two spike mark samples into human player zone
                        Commands.fastPath(follower, Paths.fiveSpecimen_pushes),

                        // SPECIMEN CYCLING ==========================================================
                        // Do the four specimens
                        dump(1),
                        dump(2),
                        dump(3),
                        dump(4)
                )
        );
    }
}