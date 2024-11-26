package org.firstinspires.ftc.teamcode.auto.opmodes;

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

@Autonomous(name = "5 Specimen", group = "MAIN")
public class FiveSpecimen extends CommandOpMode {
    public PathChain chain;
    public Follower follower;
    public AutonomousRobot robot;

    @Override
    public void initialize() {
        this.follower = new Follower(hardwareMap);
        this.robot = new AutonomousRobot(telemetry, hardwareMap, follower);
        this.follower.setStartingPose(new Pose(0, 0, Math.PI));

        double movementSpeedToIntake = 0.5;

        schedule(
            new RunCommand(robot::update),
            new SequentialCommandGroup(
                    Commands.sleepUntil(this::opModeIsActive),

                    // Dump preloaded specimen
                    Commands.followPath(follower, Paths.fiveSpecimen_initial).alongWith(
                            Commands.RaiseSlidesForSpecimenDump(robot)
                    ),
                    Commands.ClipSpecimen(robot),

                    // Push two spike mark samples into human player zone
                    Commands.followPath(follower, Paths.fiveSpecimen_pushes),

                    // SPECIMEN CYCLING ==========================================================
                    // Do the four specimens
                    Commands.followPath(follower, Paths.fiveSpecimen_intake(1)).alongWith(
                        Commands.ExtendIntakeToGripSpecimen(robot)
                    ),
                    Commands.fastPath(follower, Paths.fiveSpecimen_driveOntoSpecimen).setSpeed(movementSpeedToIntake),
                    Commands.GrabGameObjectWithIntake(robot),
                    Commands.RetractIntakeForTransfer(robot),
                    Commands.followPath(follower, Paths.fiveSpecimen_outtake(1)).alongWith(
                            Commands.RaiseSlidesForSpecimenDump(robot)
                    ),
                    Commands.ClipSpecimen(robot)
            )
        );
    }
}