package org.firstinspires.ftc.teamcode.core;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.core.params.RobotParameters;
import org.firstinspires.ftc.teamcode.core.state.intake.Intake;
import org.firstinspires.ftc.teamcode.core.state.intake.IntakeState;
import org.firstinspires.ftc.teamcode.core.state.outtake.OuttakeState;

public class Servos {
    public Servo bucketServo;
    public CRServo armServo;

    public Servo leftIntakeLiftServo;
    public Servo rightIntakeLiftServo;
    public CRServo intakeServoA;
    public CRServo intakeServoB;

    public ServoPositions positions;

    public Servos(HardwareMap hardwareMap) {
        bucketServo = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.bucketServo);
        armServo = new CRServo(hardwareMap, RobotParameters.Motors.HardwareMapNames.armServo);
        intakeServoA = new CRServo(hardwareMap, RobotParameters.Motors.HardwareMapNames.intakeServoA);
        intakeServoB = new CRServo(hardwareMap, RobotParameters.Motors.HardwareMapNames.intakeServoB);
        leftIntakeLiftServo = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.leftIntakeLiftServo);
        rightIntakeLiftServo = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.rightIntakeLiftServo);

        positions = new ServoPositions();
    }

    public class ServoPositions {
        public double armServo = 0.0;
    }

    public void setPositions(OuttakeState outtakeState, IntakeState intakeState, Motors motors) {
        if (outtakeState == OuttakeState.Down || outtakeState == OuttakeState.Folded) {
            bucketServo.setPosition(0.0);
            if (intakeState == IntakeState.Depositing || intakeState == IntakeState.Dropping) {
                positions.armServo = 0.15;
            } else {
                positions.armServo = 0.0;
            }
        } else {
            if (motors.leftIntakeSlide.getCurrentPosition() > 35.0) {
                if (outtakeState == OuttakeState.Deposit) {
                    bucketServo.setPosition(0.0);
                } else {
                    bucketServo.setPosition(0.2);
                }
            } else {
                bucketServo.setPosition(0.0);
            }
            positions.armServo = 1.0;
        }
    }

    public void setPowers(IntakeState intakeState, double intakePower) {
        if (intakeState == IntakeState.Collecting) {
            intakeServoA.set(intakePower);
            intakeServoB.set(intakePower);
        } else if (intakeState == IntakeState.Dropping) {
            intakeServoA.set(-1.0);
            intakeServoB.set(-1.0);
        } else {
            intakeServoA.set(0.0);
            intakeServoB.set(0.0);
        }
    }
}
