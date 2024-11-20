package org.firstinspires.ftc.teamcode.auto;

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

    public void update() {
        robot.drivetrain.moveIntake(0.5);
        robot.drivetrain.moveOuttake();
        robot.drivetrain.servos.setPositions(robot.state.outtake.outtakeState, robot.state.intake.intakeState, robot.drivetrain.motors, 0.64, 0.0);
        robot.drivetrain.motors.setOtherPowers();
    }

    public void extendIntake() {
        robot.state.intake.intakeState = IntakeState.ExtendedGrabbingOffWallClawOpen;
        timer.resetTimer();
    }

    public boolean isIntakeExtended() {
        return robot.drivetrain.motors.intakePosition() > RobotParameters.SlideBounds.intakeExtended - 30.0 && robot.state.intake.intakeState == IntakeState.ExtendedGrabbingOffWallClawOpen && timer.getElapsedTimeSeconds() > 2.0;
    }

    public void closeIntakeClaw() {
        robot.state.intake.intakeState = IntakeState.ExtendedGrabbingOffWallClawShut;
        timer.resetTimer();
    }

    public boolean isGrabbingDone() {
        return timer.getElapsedTimeSeconds() > 1.0;
    }

    public void retractIntake() {
        robot.state.intake.intakeState = IntakeState.Transfer;
    }

    public boolean isTransferReady() {
        boolean result =  robot.drivetrain.motors.intakePosition() < 10.0 && robot.state.intake.intakeState == IntakeState.Transfer;
        if (result) {
            robot.state.outtake.outtakeState = OuttakeState.DownClawShut;
            robot.state.intake.intakeState = IntakeState.Retracted;
            timer.resetTimer();
        }
        return result;
    }

    public boolean waitForTransfer() {
        return timer.getElapsedTimeSeconds() > 0.2;
    }

    public void outtakeStageOne() {
        timer.resetTimer();
        robot.state.outtake.outtakeState = OuttakeState.UpWithSpecimenWaitingToFlip;
    }

    public boolean outtakeStageReady() {
        return timer.getElapsedTimeSeconds() > 1.5;
    }

    public void outtakeStageTwo() {
        timer.resetTimer();
        robot.state.outtake.outtakeState = OuttakeState.UpWithSpecimenFlipped;
    }

    public void specimenClip() {
        timer.resetTimer();
        robot.state.outtake.outtakeState = OuttakeState.UpWithSpecimenOnBar;
    }

    public void outtakeStageThree() {
        timer.resetTimer();
        robot.state.outtake.outtakeState = OuttakeState.UpWithSpecimentGoingDown;
    }

    public void endSpecimenCycle() {
        robot.state.outtake.outtakeState = OuttakeState.DownClawOpen;
    }

    public boolean specimenCycleDone() {
        return robot.drivetrain.motors.outtakePosition() < 50.0 && robot.state.outtake.outtakeState == OuttakeState.DownClawOpen;
    }
}
