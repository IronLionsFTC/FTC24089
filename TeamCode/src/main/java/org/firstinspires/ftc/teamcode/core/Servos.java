package org.firstinspires.ftc.teamcode.core.state;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.core.params.RobotParameters;

public class Servos {
    public CRServo bucketServo;
    public CRServo armServo;

    public Servos(HardwareMap hardwareMap) {
        bucketServo = new CRServo(hardwareMap, RobotParameters.Motors.HardwareMapNames.bucketServo);
        armServo = new CRServo(hardwareMap, RobotParameters.Motors.HardwareMapNames.armServo);
    }
}
