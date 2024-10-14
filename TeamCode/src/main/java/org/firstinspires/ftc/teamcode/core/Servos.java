package org.firstinspires.ftc.teamcode.core;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.core.params.RobotParameters;
import org.firstinspires.ftc.teamcode.core.state.intake.IntakeState;
import org.firstinspires.ftc.teamcode.core.state.outtake.OuttakeState;

public class Servos {
    public Servo bucketServo;
    public Servo armServo;

    public Servo leftIntakeLiftServo;
    public Servo rightIntakeLiftServo;
    public CRServo intakeServoA;
    public CRServo intakeServoB;

    public ServoPositions positions;

    public Servos(HardwareMap hardwareMap) {
        bucketServo = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.bucketServo);
        armServo = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.armServo);
        intakeServoA = new CRServo(hardwareMap, RobotParameters.Motors.HardwareMapNames.intakeServoA);
        intakeServoB = new CRServo(hardwareMap, RobotParameters.Motors.HardwareMapNames.intakeServoB);
        leftIntakeLiftServo = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.leftIntakeLiftServo);
        rightIntakeLiftServo = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.rightIntakeLiftServo);

        // Reverse one of the intake lift motors.
        //rightIntakeLiftServo.setDirection(Servo.Direction.REVERSE);
        positions = new ServoPositions();
    }

    public class ServoPositions {
        public double armServo = RobotParameters.ServoBounds.armServoLower;
        public double armCurrent = armServo;
    }

    public void setPositions(OuttakeState outtakeState) {
        armServo.setPosition(positions.armCurrent);
        if (outtakeState == OuttakeState.Up || outtakeState == OuttakeState.Down) {
            bucketServo.setPosition(positions.armCurrent * 0.5 + 0.5);
        } else if (outtakeState == OuttakeState.Deposit) {
            bucketServo.setPosition(0.5);
        } else {
            bucketServo.setPosition(0.0);
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
