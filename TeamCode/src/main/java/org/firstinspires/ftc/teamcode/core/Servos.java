package org.firstinspires.ftc.teamcode.core;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.core.params.RobotParameters;
import org.firstinspires.ftc.teamcode.core.state.ComputerVision;
import org.firstinspires.ftc.teamcode.core.state.intake.IntakeState;
import org.firstinspires.ftc.teamcode.core.state.outtake.OuttakeState;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class Servos {
	// Define motors
    public CachedServo intakeLiftServo;
    public CachedServo intakeYawServo;
    public CachedServo intakeClawServo;
    public CachedServo outtakeClawServo;
    public CachedServo leftArmServo;
    public CachedServo rightArmServo;
    public CachedServo latchServo;
    public Timer transferTimer = new Timer();
    public double intakeOverridePower = 0.0;

	// Initialize the motors with hardwaremap
    public Servos(HardwareMap hardwareMap) {
        intakeLiftServo = new CachedServo(hardwareMap, RobotParameters.Motors.HardwareMapNames.intakeLiftServo);
        intakeYawServo = new CachedServo(hardwareMap, RobotParameters.Motors.HardwareMapNames.intakeYawServo);
        intakeClawServo = new CachedServo(hardwareMap, RobotParameters.Motors.HardwareMapNames.intakeClawServo);
        outtakeClawServo = new CachedServo(hardwareMap, RobotParameters.Motors.HardwareMapNames.outtakeClawServo);
        leftArmServo = new CachedServo(hardwareMap, RobotParameters.Motors.HardwareMapNames.leftArmServo);
        rightArmServo = new CachedServo(hardwareMap, RobotParameters.Motors.HardwareMapNames.rightArmServo);
        latchServo = new CachedServo(hardwareMap, RobotParameters.Motors.HardwareMapNames.latchServo);

        intakeClawServo.setPosition(RobotParameters.ServoBounds.clawWideOpen);
        outtakeClawServo.setPosition(RobotParameters.ServoBounds.outtakeClawClosed);
    }

    public void setPositions(OuttakeState outtakeState, IntakeState intakeState, Motors motors, double intakeYaw, double sampleOffset, boolean auto) {
        double intakeLift = RobotParameters.ServoBounds.intakeFolded;
        double intakeClaw = RobotParameters.ServoBounds.clawWideOpen;
        double outtakeLift = RobotParameters.ServoBounds.armDown;
        double outtakeClaw = RobotParameters.ServoBounds.clawOpen;
        double yaw = RobotParameters.ServoBounds.intakeYawZero;

        switch (intakeState) {
            case ExtendedClawDown: case ExtendedClawOpen:
                intakeLift = RobotParameters.ServoBounds.intakeDown;
                yaw = intakeYaw + sampleOffset;
                break;
            case Grabbing: case ExtendedClawShut:
                intakeLift = RobotParameters.ServoBounds.intakeDown;
                intakeClaw = RobotParameters.ServoBounds.clawClosed;
                yaw = intakeYaw;
                break;
            case Transfer:
                intakeClaw = RobotParameters.ServoBounds.clawClosed;
                break;
            case ExtendedGrabbingOffWallClawOpen:
                intakeLift = RobotParameters.ServoBounds.intakeGrabOffWall;
                yaw = RobotParameters.ServoBounds.intakeYawFlipped;
                break;
            case ExtendedGrabbingOffWallClawShut:
                intakeLift = RobotParameters.ServoBounds.intakeGrabOffWall;
                yaw = RobotParameters.ServoBounds.intakeYawFlipped;
                intakeClaw = RobotParameters.ServoBounds.clawClosed;
                break;
        }

        switch (outtakeState) {
            case DownClawShut: case UpWaitingToFlip: case UpWithSpecimenWaitingToFlip:
                outtakeClaw = RobotParameters.ServoBounds.outtakeClawClosed;
                break;
            case UpFlipped: case UpWithSpecimenFlipped: case UpWithSpecimenOnBar:
                outtakeClaw = RobotParameters.ServoBounds.outtakeClawClosed;
                outtakeLift = RobotParameters.ServoBounds.armUp + 0.12;
                break;
            case UpClawOpen: case UpWithSpecimentGoingDown:
                outtakeLift = RobotParameters.ServoBounds.armUp;
                break;
        }

        if (outtakeState != OuttakeState.DownClawOpen && outtakeState != OuttakeState.DownClawShut) {
            intakeClaw = RobotParameters.ServoBounds.clawOpen;
        }

        if (intakeState == IntakeState.Retracted || intakeState == IntakeState.Transfer) {
            if (motors.intakePosition() < 20.0) {
                latchServo.setPosition(RobotParameters.ServoBounds.latchShut);
            } else {
                latchServo.setPosition(RobotParameters.ServoBounds.latchOpen);
            }
        } else {
            latchServo.setPosition(RobotParameters.ServoBounds.latchOpen);
        }

        outtakeClawServo.setPosition(outtakeClaw);
        rightArmServo.setPosition(outtakeLift);
        leftArmServo.setPosition(1.0 - outtakeLift);
        intakeLiftServo.setPosition(intakeLift);
        intakeClawServo.setPosition(intakeClaw);
        intakeYawServo.setPosition(yaw);
    }
}
