package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.core.params.RobotParameters;
import org.firstinspires.ftc.teamcode.core.state.Team;
import org.firstinspires.ftc.teamcode.core.state.intake.IntakeState;
import org.firstinspires.ftc.teamcode.core.state.outtake.OuttakeState;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class AutonomousRobot {
    public Robot robot;
    public Timer timer = new Timer();
    public Follower follower;

    public AutonomousRobot(Telemetry t, HardwareMap hardwareMap, Follower f) {
        robot = new Robot(hardwareMap, t, null, null, Team.Red);
        follower = f;
    }

    public boolean isHalfWayThere() {
        return follower.getCurrentTValue() > 0.5;
    }

    public boolean isAtEndOfPath() {
        return follower.getCurrentTValue() > 0.99;
    }

    public boolean isAtEndOfPathAndNotMoving() {
        return isAtEndOfPath() && follower.getVelocityMagnitude() < 0.01;
    }

    @Config
    public static class AutonomousTune {
        public static double outtakeHeightToCancel = 430.0;
    }

    public void update() {
        if (robot.state.outtake.outtakeState == OuttakeState.UpWithSpecimentGoingDown) {
            if (timer.getElapsedTimeSeconds() > 0.5) robot.state.outtake.outtakeState = OuttakeState.DownClawOpen; // Automatically finish cycle
        }
        follower.update();
        robot.telemetry.update();
        robot.drivetrain.moveIntake(0.5);
        robot.drivetrain.moveOuttake(0.8);
        robot.drivetrain.servos.setPositions(robot.state.outtake.outtakeState, robot.state.intake.intakeState, robot.drivetrain.motors, 0.64, 0.0);
        robot.drivetrain.motors.setOtherPowers();
    }

    public void logDouble(String description, double value) {
        robot.telemetry.addData(description, value);
    }

    public void logBool(String description, boolean value) {
        robot.telemetry.addData(description, value);
    }

    public void extendIntakeForSpecimen() {
        robot.state.intake.intakeState = IntakeState.ExtendedGrabbingOffWallClawOpen;
        timer.resetTimer();
    }

    public void extendIntakeForSample() {
        robot.state.intake.intakeState = IntakeState.ExtendedClawDown;
        timer.resetTimer();
    }

    public boolean isIntakeExtended() {
        if (robot.drivetrain.motors.intakePosition() > RobotParameters.SlideBounds.intakeExtended - 50.0 && (robot.state.intake.intakeState == IntakeState.ExtendedGrabbingOffWallClawOpen || robot.state.intake.intakeState == IntakeState.ExtendedClawDown)) {
            timer.resetTimer();
            return true;
        }
        return false;
    }

    public void closeIntakeClaw() {
        if (robot.state.intake.intakeState == IntakeState.ExtendedGrabbingOffWallClawOpen) robot.state.intake.intakeState = IntakeState.ExtendedGrabbingOffWallClawShut;
        else robot.state.intake.intakeState = IntakeState.Grabbing;
    }

    public boolean isIntakeDoneGrabbing() {
        if (timer.getElapsedTimeSeconds() > 0.6 && robot.state.intake.intakeState == IntakeState.ExtendedClawDown) closeIntakeClaw();
        if (timer.getElapsedTimeSeconds() > 1.0 && robot.state.intake.intakeState == IntakeState.ExtendedGrabbingOffWallClawOpen) closeIntakeClaw();
        if (timer.getElapsedTimeSeconds() > 1.0 && robot.state.intake.intakeState == IntakeState.Grabbing) {
            if (robot.state.intake.foldIntakeBeforeRetraction.getElapsedTimeSeconds() > 2.6) {
                robot.state.intake.foldIntakeBeforeRetraction.resetTimer();
            }
            robot.state.intake.intakeState = IntakeState.Transfer;
            return true;
        }
        if (timer.getElapsedTimeSeconds() > 2.0 && robot.state.intake.intakeState == IntakeState.ExtendedGrabbingOffWallClawShut) {
            if (robot.state.intake.foldIntakeBeforeRetraction.getElapsedTimeSeconds() > 2.6) {
                robot.state.intake.foldIntakeBeforeRetraction.resetTimer();
            }
            robot.state.intake.intakeState = IntakeState.Transfer;
            return true;
        }
        return false;
    }

    public boolean isTransferReady() {
        return robot.drivetrain.motors.intakePosition() < 20.0;
    }

    public boolean isClipDone() {
        if (timer.getElapsedTimeSeconds() > 3.0 || robot.drivetrain.motors.outtakePosition() > AutonomousTune.outtakeHeightToCancel) {
            robot.state.outtake.outtakeState = OuttakeState.UpWithSpecimentGoingDown; // Set state here to be idiot proof [will always stop the slides]
            timer.resetTimer();
            return true;
        }
        return false;
    }

    public boolean waitForTransfer() {
        return timer.getElapsedTimeSeconds() > 2.5;
    }

    public boolean specimenStageOne() {
        if (timer.getElapsedTimeSeconds() > 0.3) {
            timer.resetTimer();
            robot.state.outtake.outtakeState = OuttakeState.UpWithSpecimenWaitingToFlip;
            return true;
        }
        return false;
    }

    public boolean outtakeStageReady() {
        return timer.getElapsedTimeSeconds() > 1.5;
    }

    public void specimenStageTwo() {
        timer.resetTimer();
        robot.state.outtake.outtakeState = OuttakeState.UpWithSpecimenFlipped;
    }

    public void specimenClip() {
        timer.resetTimer();
        robot.state.outtake.outtakeState = OuttakeState.UpWithSpecimenOnBar;
    }

    public void specimenStageThree() {
        timer.resetTimer();
        robot.state.outtake.outtakeState = OuttakeState.UpWithSpecimentGoingDown;
    }

    public void endSpecimenCycle() {
        robot.state.outtake.outtakeState = OuttakeState.DownClawOpen;
    }

    public boolean specimenCycleDone() {
        return robot.drivetrain.motors.outtakePosition() < 50.0 && robot.state.outtake.outtakeState == OuttakeState.DownClawOpen;
    }

    public void raiseSlidesForSampleDump () {
        robot.state.outtake.outtakeState = OuttakeState.DownClawShut;
        timer.resetTimer();
    }

    public void raiseSlidesForSpecimenDump () {
        raiseSlidesForSampleDump(); // Same first step, just named differently to avoid confusion
    }

    public boolean areSlidesReadyForSampleDump() {
        if (timer.getElapsedTimeSeconds() > 0.3) {
            robot.state.outtake.outtakeState = OuttakeState.UpWaitingToFlip;
        }
        return robot.state.outtake.outtakeState == OuttakeState.UpWaitingToFlip && robot.drivetrain.motors.outtakePosition() > RobotParameters.SlideBounds.outtakeUp - 200.0;
    }

    public boolean areSlidesReadyForSpecimenDump() {
        if (timer.getElapsedTimeSeconds() > 0.3) {
            robot.state.outtake.outtakeState = OuttakeState.UpWithSpecimenWaitingToFlip;
        }
        if (timer.getElapsedTimeSeconds() > 0.7) {
            robot.state.outtake.outtakeState = OuttakeState.UpWithSpecimenFlipped;
        }
        return timer.getElapsedTimeSeconds() > 1.5;
    }

    public void performDump() {
        timer.resetTimer();
        robot.state.outtake.outtakeState = OuttakeState.UpFlipped;
    }

    public boolean isDumpDone() {
        if (timer.getElapsedTimeSeconds() > 0.5) {
            robot.state.outtake.outtakeState = OuttakeState.UpClawOpen;
        }
        if (timer.getElapsedTimeSeconds() > 0.7) {
            robot.state.outtake.outtakeState = OuttakeState.UpWaitingToGoDown;
        }
        if (timer.getElapsedTimeSeconds() > 1.2) {
            robot.state.outtake.outtakeState = OuttakeState.DownClawOpen;
            return true;
        }
        return false;
    }
}
