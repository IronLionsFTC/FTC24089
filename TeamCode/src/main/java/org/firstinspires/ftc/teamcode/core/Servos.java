package org.firstinspires.ftc.teamcode.core.state;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.core.params.RobotParameters;

public class Servos {
    public CRServo leftIntakeServo;
    public CRServo rightIntakeServo;

    public Servos(HardwareMap hardwareMap) {
        leftIntakeServo = new CRServo(hardwareMap, RobotParameters.Motors.HardwareMapNames.leftIntakeServo);
        rightIntakeServo = new CRServo(hardwareMap, RobotParameters.Motors.HardwareMapNames.rightIntakeServo);
    }
}
