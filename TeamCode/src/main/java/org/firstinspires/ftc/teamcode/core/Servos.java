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
    public Servo armServo;
    public Servo leftIntakeLiftServo;
    public Servo rightIntakeLiftServo;
    public CRServo intakeServoA;
    public CRServo intakeServoB;

	// Initialize the motors with hardwaremap
    public Servos(HardwareMap hardwareMap) {
        bucketServo = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.bucketServo);
        armServo = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.armServo);
        intakeServoA = new CRServo(hardwareMap, RobotParameters.Motors.HardwareMapNames.intakeServoA);
        intakeServoB = new CRServo(hardwareMap, RobotParameters.Motors.HardwareMapNames.intakeServoB);
        leftIntakeLiftServo = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.leftIntakeLiftServo);
        rightIntakeLiftServo = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.rightIntakeLiftServo);
    }

    public void setPositions(OuttakeState outtakeState, IntakeState intakeState, Motors motors) {
        if (outtakeState == OuttakeState.Down || outtakeState == OuttakeState.Folded) {
            bucketServo.setPosition(RobotParameters.ServoBounds.bucketOpen);
            if (intakeState == IntakeState.Depositing || intakeState == IntakeState.Dropping) {
                armServo.setPosition(RobotParameters.ServoBounds.armTransfer);
            } else {
                armServo.setPosition(RobotParameters.ServoBounds.armDown);
            }
        } else if (outtakeState == OuttakeState.Deposit || outtakeState == OuttakeState.Up) {
            if (outtakeState == OuttakeState.Deposit) {
                bucketServo.setPosition(RobotParameters.ServoBounds.bucketTransfer); // Give some leeway to make sure it doesn't get stuck
            } else if (motors.leftIntakeSlide.getCurrentPosition() > RobotParameters.SlideBounds.intakeClearance - 10.0 || motors.leftOuttakeSlide.getCurrentPosition() > 200.0) {
                bucketServo.setPosition(RobotParameters.ServoBounds.bucketClosed);
            } else {
                bucketServo.setPosition(RobotParameters.ServoBounds.bucketTransfer);
            }
            if (motors.leftIntakeSlide.getCurrentPosition() > RobotParameters.SlideBounds.intakeClearance - 10.0 || motors.leftOuttakeSlide.getCurrentPosition() > 200.0) {
                armServo.setPosition(RobotParameters.ServoBounds.armUp);
            } else {
                armServo.setPosition(RobotParameters.ServoBounds.armTransfer);
            }
        }
    }

    public void setPowers(IntakeState intakeState, double intakePower, Sensors sensors) {
        if (intakeState == IntakeState.Collecting) {
            intakeServoA.set(intakePower);
            intakeServoB.set(intakePower);
        } else if (intakeState == IntakeState.Dropping) {
            intakeServoA.set(-0.3);
            intakeServoB.set(-0.3);
        } else if (intakeState == IntakeState.Depositing && sensors.d() > RobotParameters.Thresholds.intakeSamplePresent) {
            intakeServoA.set(-RobotParameters.PIDConstants.reverseIntakeSpeed);
            intakeServoB.set(-RobotParameters.PIDConstants.reverseIntakeSpeed);
        } else {
            intakeServoA.set(0.0);
            intakeServoB.set(0.0);
        }
    }
}
