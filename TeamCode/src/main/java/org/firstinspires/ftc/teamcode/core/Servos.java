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

    public CRServo intakeServoA;
    public CRServo intakeServoB;

    public ServoPositions positions;

    public Servos(HardwareMap hardwareMap) {
        bucketServo = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.bucketServo);
        armServo = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.armServo);
        intakeServoA = hardwareMap.get(CRServo.class, RobotParameters.Motors.HardwareMapNames.intakeServoA);
        intakeServoB = hardwareMap.get(CRServo.class, RobotParameters.Motors.HardwareMapNames.intakeServoB);
        positions = new ServoPositions();
    }

    public class ServoPositions {
        public double armServo = RobotParameters.ServoBounds.armServoLower;
        public double bucketServo = 0.0;
        public double armCurrent = armServo;
        public double armError = 0.0;
    }

    public void setPositions(OuttakeState outtakeState) {
        positions.armServo = RobotParameters.ServoBounds.armServoLower;
        armServo.setPosition(positions.armCurrent);
        if (outtakeState == OuttakeState.Down || outtakeState == OuttakeState.Up) {
            double bucketPos = positions.armCurrent * 0.6;
            if (bucketPos > RobotParameters.ServoBounds.bucketServoLower) {
                bucketPos = RobotParameters.ServoBounds.bucketServoLower;
            }
            bucketServo.setPosition(bucketPos);
        } else if (outtakeState == OuttakeState.Deposit) {
            bucketServo.setPosition(0.0);
        } else {
            bucketServo.setPosition(-0.4);
        }
    }

    public void setPowers(IntakeState intakeState) {
        if (intakeState == IntakeState.Collecting) {
            intakeServoA.set(1.0);
            intakeServoB.set(1.0);
        } else {
            intakeServoA.set(0.0);
            intakeServoB.set(0.0);
        }
    }
}
