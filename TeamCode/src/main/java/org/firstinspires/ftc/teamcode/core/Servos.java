package org.firstinspires.ftc.teamcode.core;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.core.params.RobotParameters;
import org.firstinspires.ftc.teamcode.core.state.intake.IntakeState;
import org.firstinspires.ftc.teamcode.core.state.outtake.OuttakeState;

public class Servos {
	// Define motors
    public Servo intakeLiftServo;
    public Servo intakeYawServo;
    public Servo intakeClawServo;
    public double intakeOverridePower = 0.0;

	// Initialize the motors with hardwaremap
    public Servos(HardwareMap hardwareMap) {
        intakeLiftServo = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.intakeLiftServo);
        intakeYawServo = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.intakeYawServo);
        intakeClawServo = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.intakeClawServo);
    }

    public void setPositions(OuttakeState outtakeState, IntakeState intakeState, Motors motors, double intakeYaw) {
        double lift = RobotParameters.ServoBounds.intakeFolded;
        double claw = RobotParameters.ServoBounds.clawOpen;
        double yaw = RobotParameters.ServoBounds.intakeYawZero;

        switch (intakeState) {
            case ExtendedClawDown:
                lift = RobotParameters.ServoBounds.intakeDown;
                yaw = intakeYaw;
                break;
            case Grabbing:
                lift = RobotParameters.ServoBounds.intakeDown;
                claw = RobotParameters.ServoBounds.clawClosed;
                yaw = intakeYaw;
                break;
            case Transfer:
                claw = RobotParameters.ServoBounds.clawClosed;
                break;
        }
        intakeLiftServo.setPosition(lift);
        intakeClawServo.setPosition(claw);
        intakeYawServo.setPosition(yaw);
    }
}
