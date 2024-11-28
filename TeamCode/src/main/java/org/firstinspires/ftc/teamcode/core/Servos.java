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
    public Servo intakeLiftServo;
    public Servo intakeYawServo;
    public Servo intakeClawServo;
    public Servo outtakeClawServo;
    public Servo leftArmServo;
    public Servo rightArmServo;
    public Servo latchServo;
    public Timer transferTimer = new Timer();
    public double intakeOverridePower = 0.0;

	// Initialize the motors with hardwaremap
    public Servos(HardwareMap hardwareMap) {
        intakeLiftServo = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.intakeLiftServo);
        intakeYawServo = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.intakeYawServo);
        intakeClawServo = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.intakeClawServo);
        outtakeClawServo = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.outtakeClawServo);
        leftArmServo = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.leftArmServo);
        rightArmServo = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.rightArmServo);
        latchServo = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.latchServo);
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
                outtakeClaw = RobotParameters.ServoBounds.clawClosed;
                break;
            case UpFlipped: case UpWithSpecimenFlipped: case UpWithSpecimenOnBar:
                outtakeClaw = RobotParameters.ServoBounds.clawClosed;
                outtakeLift = RobotParameters.ServoBounds.armUp + 0.07;
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
