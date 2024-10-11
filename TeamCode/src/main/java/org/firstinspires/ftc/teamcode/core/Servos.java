package org.firstinspires.ftc.teamcode.core.state;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.core.params.RobotParameters;

public class Servos {
    public Servo bucketServo;
    public Servo armServo;

    public CRServo intakeServoA;
    public CRServo intakeServoB;

    public ServoPositions positions;
    public CRServoPowers powers;

    public Servos(HardwareMap hardwareMap) {
        bucketServo = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.bucketServo);
        armServo = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.armServo);
        intakeServoA = hardwareMap.get(CRServo.class, RobotParameters.Motors.HardwareMapNames.intakeServoA);
        intakeServoB = hardwareMap.get(CRServo.class, RobotParameters.Motors.HardwareMapNames.intakeServoB);
        positions = new ServoPositions();
        powers = new CRServoPowers();
    }

    public class ServoPositions {
        public double armServo = RobotParameters.ServoBounds.armServoLower;
        public double bucketServo = 0.0;
        public double armCurrent = armServo;
        public double armError = 0.0;
    }

    public class CRServoPowers {
        public double intake = 0.0;
    }
}
