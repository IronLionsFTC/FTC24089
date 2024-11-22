package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.core.params.RobotParameters;
import org.firstinspires.ftc.teamcode.core.state.Team;
import org.firstinspires.ftc.teamcode.core.state.intake.IntakeState;
import org.firstinspires.ftc.teamcode.core.state.outtake.OuttakeState;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class AutonomousRobot {
    public Robot robot;
    public Timer timer = new Timer();

    public AutonomousRobot(Telemetry t, HardwareMap hardwareMap) {
        robot = new Robot(hardwareMap, t, null, null, Team.Red);
    }

    @Config
    public static class AutonomousTune {
        public static double outtakeHeightToCancel = 430.0;
    }

    public void update() {
        robot.drivetrain.moveIntake(0.5);
        robot.drivetrain.moveOuttake(0.8);
        robot.drivetrain.servos.setPositions(robot.state.outtake.outtakeState, robot.state.intake.intakeState, robot.drivetrain.motors, 0.64, 0.0);
        robot.drivetrain.motors.setOtherPowers();
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
        return robot.drivetrain.motors.intakePosition() > RobotParameters.SlideBounds.intakeExtended - 50.0 && (robot.state.intake.intakeState == IntakeState.ExtendedGrabbingOffWallClawOpen || robot.state.intake.intakeState == IntakeState.ExtendedClawDown) && timer.getElapsedTimeSeconds() > 2.0;
    }

    public void closeIntakeClaw() {
        if (robot.state.intake.intakeState == IntakeState.ExtendedGrabbingOffWallClawOpen) robot.state.intake.intakeState = IntakeState.ExtendedGrabbingOffWallClawShut;
        else robot.state.intake.intakeState = IntakeState.Grabbing;
        timer.resetTimer();
    }

    public boolean isIntakeDone() {
        if (timer.getElapsedTimeSeconds() > 0.1) {
            if (robot.state.intake.foldIntakeBeforeRetraction.getElapsedTimeSeconds() > 2.0) {
                robot.state.intake.foldIntakeBeforeRetraction.resetTimer();
            }
            robot.state.intake.intakeState = IntakeState.Transfer;
        }

        if (robot.drivetrain.motors.intakePosition() < 15.0) {
            timer.resetTimer();
            return true;
        } else return false;
    }

    public boolean isGrabbingDone() {
        return timer.getElapsedTimeSeconds() > 0.1;
    }

    public void retractIntake() {
        robot.state.intake.intakeState = IntakeState.Transfer;
        robot.state.intake.foldIntakeBeforeRetraction.resetTimer();
    }

    public boolean isTransferReady() {
        boolean result =  robot.drivetrain.motors.intakePosition() < 15.0 && robot.state.intake.intakeState == IntakeState.Transfer && timer.getElapsedTimeSeconds() > 0.1;
        if (result) {
            robot.state.outtake.outtakeState = OuttakeState.DownClawShut;
            robot.state.intake.intakeState = IntakeState.Retracted;
            timer.resetTimer();
        }
        return result;
    }

    public boolean waitForClip() {
        return timer.getElapsedTimeSeconds() > 3.0 || robot.drivetrain.motors.outtakePosition() > AutonomousTune.outtakeHeightToCancel;
    }

    public boolean waitForTransfer() {
        return timer.getElapsedTimeSeconds() > 0.5;
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

    public boolean raiseSlidesForSampleDump () {
        robot.state.outtake.outtakeState = OuttakeState.DownClawShut;
        if (timer.getElapsedTimeSeconds() > 1.0) {
            robot.state.outtake.outtakeState = OuttakeState.UpWaitingToFlip;
            return true;
        }
        return false;
    }

    public boolean areSlidesReadyForSampleDump() {
        return robot.state.outtake.outtakeState == OuttakeState.UpWaitingToFlip && robot.drivetrain.motors.outtakePosition() > RobotParameters.SlideBounds.outtakeUp - 200.0;
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