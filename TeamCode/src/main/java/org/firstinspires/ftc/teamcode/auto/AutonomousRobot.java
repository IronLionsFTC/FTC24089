package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.core.params.RobotParameters;
import org.firstinspires.ftc.teamcode.core.state.Team;
import org.firstinspires.ftc.teamcode.core.state.intake.Intake;
import org.firstinspires.ftc.teamcode.core.state.intake.IntakeState;
import org.firstinspires.ftc.teamcode.core.state.outtake.OuttakeState;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class AutonomousRobot extends SubsystemBase {
    public Robot robot;
    public Timer intakeTimer = new Timer();
    public Timer outtakeTimer = new Timer();
    public Follower follower;
    public double clawPos = RobotParameters.ServoBounds.intakeYawZero;

    public AutonomousRobot(Telemetry t, HardwareMap hardwareMap, Follower f) {
        robot = new Robot(hardwareMap, t, null, null, Team.Red);
        follower = f;
    }

    @Override
    public void periodic() {
        this.update();
    }

    public boolean isHalfWayThere() {
        return follower.getCurrentTValue() > 0.5;
    }

    public boolean atPathEnd() {
        return follower.getCurrentTValue() > 0.99;
    }

    public boolean isAtEndOfPathAndNotMoving() {
        return atPathEnd() && follower.getVelocityMagnitude() < 0.01;
    }

    @Config
    public static class AutonomousTune {
        public static double outtakeHeightToCancel = 400.0;
    }

    public void update() {
        if (robot.state.outtake.outtakeState == OuttakeState.UpWithSpecimentGoingDown) {
            if (outtakeTimer.getElapsedTimeSeconds() > 0.5) robot.state.outtake.outtakeState = OuttakeState.DownClawOpen; // Automatically finish cycle
        }
        follower.update();
        robot.telemetry.update();
        robot.drivetrain.moveIntake(0.5);
        robot.drivetrain.moveOuttake(0.8, true);
        robot.drivetrain.servos.setPositions(robot.state.outtake.outtakeState, robot.state.intake.intakeState, robot.drivetrain.motors, clawPos, 0.0, true);
        robot.drivetrain.motors.setOtherPowers();
    }

    public void clawTo45Degrees() {
        clawPos = RobotParameters.ServoBounds.intakeYawZero - 0.12;
    }

    public void rotateClaw(double degrees) {
        clawPos = RobotParameters.ServoBounds.intakeYawZero - 0.12 * (degrees / 45);
    }

    public void logDouble(String description, double value) {
        robot.telemetry.addData(description, value);
    }

    public void logBool(String description, boolean value) {
        robot.telemetry.addData(description, value);
    }

    public void extendIntakeForSpecimen() {
        clawPos = RobotParameters.ServoBounds.intakeYawZero;
        robot.state.intake.intakeState = IntakeState.ExtendedGrabbingOffWallClawOpen;
        intakeTimer.resetTimer();
    }

    public void extendIntakeForSample() {
        clawPos = RobotParameters.ServoBounds.intakeYawZero;
        robot.state.intake.intakeState = IntakeState.ExtendedClawDown;
        intakeTimer.resetTimer();
    }

    public boolean isIntakeExtended() {
        if (robot.drivetrain.motors.intakePosition() > RobotParameters.SlideBounds.intakeExtended - 50.0 && (robot.state.intake.intakeState == IntakeState.ExtendedGrabbingOffWallClawOpen || robot.state.intake.intakeState == IntakeState.ExtendedClawDown)) {
            if (robot.state.intake.intakeState == IntakeState.ExtendedClawDown) {
                if (intakeTimer.getElapsedTimeSeconds() > 0.6) {
                    intakeTimer.resetTimer();
                    return true;
                }
            }
            if (robot.state.intake.intakeState == IntakeState.ExtendedGrabbingOffWallClawOpen) {
                if (intakeTimer.getElapsedTimeSeconds() > 1.0) {
                    intakeTimer.resetTimer();
                    return true;
                }
            }
        }
        return false;
    }

    public void closeIntakeClaw() {
        if (robot.state.intake.intakeState == IntakeState.ExtendedGrabbingOffWallClawOpen) robot.state.intake.intakeState = IntakeState.ExtendedGrabbingOffWallClawShut;
        else robot.state.intake.intakeState = IntakeState.Grabbing;
    }

    public boolean isIntakeDoneGrabbing() {
        if (intakeTimer.getElapsedTimeSeconds() > 0.6 && robot.state.intake.intakeState == IntakeState.ExtendedClawDown) closeIntakeClaw();
        if (intakeTimer.getElapsedTimeSeconds() > 1.0 && robot.state.intake.intakeState == IntakeState.ExtendedGrabbingOffWallClawOpen) closeIntakeClaw();
        if (intakeTimer.getElapsedTimeSeconds() > 1.0 && robot.state.intake.intakeState == IntakeState.Grabbing) {
            if (robot.state.intake.foldIntakeBeforeRetraction.getElapsedTimeSeconds() > 2.6) {
                robot.state.intake.foldIntakeBeforeRetraction.resetTimer();
            }
            robot.state.intake.intakeState = IntakeState.Transfer;
            return true;
        }
        if (intakeTimer.getElapsedTimeSeconds() > 2.0 && robot.state.intake.intakeState == IntakeState.ExtendedGrabbingOffWallClawShut) {
            if (robot.state.intake.foldIntakeBeforeRetraction.getElapsedTimeSeconds() > 2.6) {
                robot.state.intake.foldIntakeBeforeRetraction.resetTimer();
            }
            robot.state.intake.intakeState = IntakeState.Transfer;
            return true;
        }
        return false;
    }

    public void retractIntake() {
        robot.state.intake.intakeState = IntakeState.Transfer;
        robot.state.intake.foldIntakeBeforeRetraction.resetTimer();
    }

    public boolean isTransferReady() {
        return robot.drivetrain.motors.intakePosition() < 15.0;
    }

    public boolean isClipDone() {
        if (outtakeTimer.getElapsedTimeSeconds() > 3.0 || robot.drivetrain.motors.outtakePosition() > AutonomousTune.outtakeHeightToCancel) {
            robot.state.outtake.outtakeState = OuttakeState.UpWithSpecimentGoingDown; // Set state here to be idiot proof [will always stop the slides]
            outtakeTimer.resetTimer();
            return true;
        }
        return false;
    }

    public void specimenClip() {
        outtakeTimer.resetTimer();
        robot.state.outtake.outtakeState = OuttakeState.UpWithSpecimenOnBar;
    }

    public void raiseSlidesForSampleDump () {
        robot.state.intake.intakeState = IntakeState.Retracted;
        robot.state.outtake.outtakeState = OuttakeState.DownClawShut;
        outtakeTimer.resetTimer();
    }

    public void raiseSlidesForSpecimenDump () {
        raiseSlidesForSampleDump(); // Same first step, just named differently to avoid confusion
    }

    public boolean areSlidesReadyForSampleDump() {
        if (outtakeTimer.getElapsedTimeSeconds() > 0.3) {
            robot.state.outtake.outtakeState = OuttakeState.UpWaitingToFlip;
        }
        return robot.state.outtake.outtakeState == OuttakeState.UpWaitingToFlip && robot.drivetrain.motors.outtakePosition() > RobotParameters.SlideBounds.outtakeUp - 200.0;
    }

    public boolean areSlidesReadyForSpecimenDump() {
        if (outtakeTimer.getElapsedTimeSeconds() > 0.3) {
            robot.state.outtake.outtakeState = OuttakeState.UpWithSpecimenWaitingToFlip;
        }
        if (outtakeTimer.getElapsedTimeSeconds() > 0.7) {
            robot.state.outtake.outtakeState = OuttakeState.UpWithSpecimenFlipped;
        }
        return outtakeTimer.getElapsedTimeSeconds() > 1.5;
    }

    public void performDump() {
        outtakeTimer.resetTimer();
        robot.state.outtake.outtakeState = OuttakeState.UpFlipped;
    }

    public boolean isDumpDone() {
        if (outtakeTimer.getElapsedTimeSeconds() > 0.5) {
            robot.state.outtake.outtakeState = OuttakeState.UpClawOpen;
        }
        if (outtakeTimer.getElapsedTimeSeconds() > 0.7) {
            robot.state.outtake.outtakeState = OuttakeState.UpWaitingToGoDown;
        }
        if (outtakeTimer.getElapsedTimeSeconds() > 1.2) {
            robot.state.outtake.outtakeState = OuttakeState.DownClawOpen;
            return true;
        }
        return false;
    }
}
