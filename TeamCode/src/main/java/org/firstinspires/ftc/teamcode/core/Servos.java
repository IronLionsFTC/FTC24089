package org.firstinspires.ftc.teamcode.core;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.core.params.RobotParameters;
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

    public void setPositions(OuttakeState outtakeState, IntakeState intakeState, Motors motors, boolean armUp, boolean retract) {
        double bucketPos = 0.0;
        double armPos = 0.0;

        if (outtakeState == OuttakeState.Down || outtakeState == OuttakeState.Folded) {
            bucketPos = RobotParameters.ServoBounds.bucketOpen;
            if (intakeState == IntakeState.Depositing || intakeState == IntakeState.Dropping) {
                armPos = RobotParameters.ServoBounds.armTransfer;
            } else {
                armPos = RobotParameters.ServoBounds.armDown;
            }
        } else if (outtakeState == OuttakeState.Deposit || outtakeState == OuttakeState.Up || outtakeState == OuttakeState.PassthroughDeposit || outtakeState == OuttakeState.Waiting) {
            if (outtakeState == OuttakeState.Deposit || outtakeState == OuttakeState.PassthroughDeposit) {
                bucketPos = 0.0;
            } else if (motors.leftIntakeSlide.getCurrentPosition() > RobotParameters.SlideBounds.intakeClearance - 10.0 || motors.leftOuttakeSlide.getCurrentPosition() > 300.0) {
                bucketPos = RobotParameters.ServoBounds.bucketClosed;
            } else if (outtakeState != OuttakeState.Waiting){
                bucketPos = RobotParameters.ServoBounds.bucketTransfer;
            }
            if (motors.leftOuttakeSlide.getCurrentPosition() > RobotParameters.SlideBounds.outtakeUp - 100.0) {
                if (outtakeState != OuttakeState.Deposit) {
                    armPos = RobotParameters.ServoBounds.armUp;
                } else {
                    armPos = RobotParameters.ServoBounds.armWait;
                }
            } else if (outtakeState == OuttakeState.Up || outtakeState == OuttakeState.Deposit) {
                armPos = RobotParameters.ServoBounds.armWait;
            }
        }
        if (outtakeState == OuttakeState.PassthroughDeposit) {
            armPos = RobotParameters.ServoBounds.armUp;
        }
        if (armUp) {
            armPos = 0.1;
        }
        if (outtakeState == OuttakeState.Waiting) {
            if (motors.leftIntakeSlide.getCurrentPosition() > RobotParameters.SlideBounds.intakeClearance - 10.0 || retract) {
                armPos = RobotParameters.ServoBounds.armWait;
                bucketPos = RobotParameters.ServoBounds.bucketClosed;
            }
        }

        if (intakeState == IntakeState.Retracted || intakeState == IntakeState.Folded) {
            if (motors.leftIntakeSlide.getCurrentPosition() < 10.0 && motors.leftOuttakeSlide.getCurrentPosition() < RobotParameters.Thresholds.outtakeHeightToRetractIntakeLower && outtakeState != OuttakeState.Up) {
                latchServo.setPosition(RobotParameters.ServoBounds.latchClosed);
            } else {
                latchServo.setPosition(RobotParameters.ServoBounds.latchOpened);
            }
        } else {
            latchServo.setPosition(RobotParameters.ServoBounds.latchOpened);
        }

        if (outtakeState == OuttakeState.LevelOneHang) {
            armPos = RobotParameters.ServoBounds.armDown - 0.05;
        }

        if (intakeState == IntakeState.Depositing || intakeState == IntakeState.Dropping) {
            armPos = RobotParameters.ServoBounds.armTransfer;
        }

        if (outtakeState == OuttakeState.Up) {
            bucketPos = RobotParameters.ServoBounds.bucketClosed;
        }

        bucketServo.setPosition(bucketPos);
        armServoA.setPosition(armPos);
        armServoB.setPosition(1.0 - armPos);
    }

    public void setPowers(IntakeState intakeState, double intakePower, Sensors sensors, boolean cancelIntake) {
        if (Math.abs(intakeOverridePower) < 0.1) {
            if (intakeState == IntakeState.Collecting) {
                intakeServoA.set(intakePower);
                intakeServoB.set(intakePower);
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
