package org.firstinspires.ftc.teamcode.core;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.core.params.RobotParameters;
import org.firstinspires.ftc.teamcode.core.state.RobotState;
import org.firstinspires.ftc.teamcode.core.state.intake.IntakeState;
import org.firstinspires.ftc.teamcode.core.state.outtake.OuttakeState;

public class Servos {
	// Define motors
    public Servo bucketServo;
    public Servo armServoB;
    public Servo armServoA;
    public Servo leftIntakeLiftServo;
    public Servo rightIntakeLiftServo;
    public Servo latchServo;

    public CRServo intakeServoA;
    public CRServo intakeServoB;
    public double intakeOverridePower = 0.0;

	// Initialize the motors with hardwaremap
    public Servos(HardwareMap hardwareMap) {
        bucketServo = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.bucketServo);
        armServoA = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.armServoA);
        armServoB = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.armServoB);
        latchServo = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.latchServo);

        intakeServoA = new CRServo(hardwareMap, RobotParameters.Motors.HardwareMapNames.intakeServoA);
        intakeServoB = new CRServo(hardwareMap, RobotParameters.Motors.HardwareMapNames.intakeServoB);

        leftIntakeLiftServo = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.leftIntakeLiftServo);
        rightIntakeLiftServo = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.rightIntakeLiftServo);
        rightIntakeLiftServo.setPosition(RobotParameters.ServoBounds.intakeFolded);
        leftIntakeLiftServo.setPosition(1.0 - RobotParameters.ServoBounds.intakeFolded);
        armServoA.setPosition(RobotParameters.ServoBounds.armDown);
        armServoB.setPosition(1.0 - RobotParameters.ServoBounds.armDown);
    }

    public void setPositions(RobotState state, Motors motors) {
        double bucketPos = 0.0;
        double armPos = 0.0;
        double outtakePos = motors.outtakePosition();
        double intakePos = motors.intakePosition();
        double latchPosition = RobotParameters.ServoBounds.latchOpened;

        // Decide Arm Position
        // When should the arm be down?
        if (state.intake.intakeState == IntakeState.Collecting || state.intake.intakeState == IntakeState.Evaluating ||
                state.intake.intakeState == IntakeState.Extended || state.intake.intakeState == IntakeState.Folded) {
            armPos = RobotParameters.ServoBounds.armDown;
            bucketPos = RobotParameters.ServoBounds.bucketOpen;
        }

        // No matter what, transfer position if waiting to transfer.
        if (state.intake.intakeState == IntakeState.Depositing || state.intake.intakeState == IntakeState.Dropping) {
            armPos = RobotParameters.ServoBounds.armTransfer;
            bucketPos = RobotParameters.ServoBounds.bucketTransfer;
        }

        // If retracted, check if normally retracted or if currently waiting to outtake.
        if (state.intake.intakeState == IntakeState.Retracted) {
            if ((intakePos > RobotParameters.SlideBounds.intakeClearance - 30.0 || state.outtake.retract) && state.outtake.outtakeState == OuttakeState.Waiting) {
                armPos = RobotParameters.ServoBounds.armWait;
                bucketPos = RobotParameters.ServoBounds.bucketClosed;
            } else {
                armPos = RobotParameters.ServoBounds.armDown;
                bucketPos = RobotParameters.ServoBounds.bucketTransfer;
            }
        }

        // Deal with raised outtake.
        if (state.outtake.outtakeState == OuttakeState.Up) {
            bucketPos = RobotParameters.ServoBounds.bucketClosed;
            if (outtakePos > RobotParameters.SlideBounds.outtakeUp - 100.0) {
                armPos = RobotParameters.ServoBounds.armUp;
            } else {
                armPos = RobotParameters.ServoBounds.armWait;
            }
        }

        // Just raise arm up when the sample is dropped.
        if (state.outtake.outtakeState == OuttakeState.Deposit) {
            bucketPos = 0.0; // Fully open, leave no room for getting it stuck.
            if (state.outtake.armRaiseTimer.getElapsedTime() > 200.0) {
                armPos = RobotParameters.ServoBounds.armUp - 0.15;
            } else {
                armPos = RobotParameters.ServoBounds.armUp;
            }
        }

        // Latch position.
        if (state.intake.intakeState == IntakeState.Retracted || state.intake.intakeState == IntakeState.Folded) {
            if (intakePos < 10.0 && outtakePos < RobotParameters.Thresholds.outtakeHeightToRetractIntakeLower && state.outtake.outtakeState != OuttakeState.Up) {
                latchServo.setPosition(RobotParameters.ServoBounds.latchClosed);
            } else {
                latchServo.setPosition(RobotParameters.ServoBounds.latchOpened);
            }
        } else {
            latchServo.setPosition(RobotParameters.ServoBounds.latchOpened);
        }

        if (state.outtake.outtakeState == OuttakeState.LevelOneHang) {
            armPos = RobotParameters.ServoBounds.armUp;
            bucketPos = RobotParameters.ServoBounds.bucketClosed;
        }

        // Apply new positions to servos.
        armServoA.setPosition(armPos);
        armServoB.setPosition(1.0 - armPos);
        bucketServo.setPosition(bucketPos);
    }

    public void setPowers(IntakeState intakeState, double intakePower, Sensors sensors, boolean cancelIntake) {
        if (Math.abs(intakeOverridePower) < 0.1) {
            if (intakeState == IntakeState.Collecting) {
                intakeServoA.set(RobotParameters.PIDConstants.intakeSpeed);
                intakeServoB.set(RobotParameters.PIDConstants.intakeSpeed);
            } else if (intakeState == IntakeState.Dropping) {
                intakeServoA.set(-RobotParameters.SystemsTuning.reverseIntakeSpeed);
                intakeServoB.set(-RobotParameters.SystemsTuning.reverseIntakeSpeed);
            } else if (intakeState == IntakeState.Depositing && sensors.d() > RobotParameters.Thresholds.intakeSamplePresent) {
                intakeServoA.set(-0.2);
                intakeServoB.set(-0.2);
            } else {
                intakeServoA.set(0.0);
                intakeServoB.set(0.0);
            }
        } else {
            intakeServoA.set(intakeOverridePower);
            intakeServoB.set(intakeOverridePower);
        }
        if (cancelIntake) {
            intakeServoA.set(0.0);
            intakeServoB.set(0.0);
        }
    }
}
