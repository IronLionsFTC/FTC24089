package org.firstinspires.ftc.teamcode.core;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import org.firstinspires.ftc.teamcode.core.params.RobotParameters;

public class Motors {
    // Drive motors
    public Motor leftFront;
    public Motor leftBack;
    public Motor rightFront;
    public Motor rightBack;

    // Intake motors
    public Motor leftIntakeSlide;
    public Motor rightIntakeSlide;

    // Outake motors
    public Motor leftOuttakeSlide;
    public Motor rightOuttakeSlide;

    // Motor Powers
    public MotorPowers powers;

    public Motors(HardwareMap hardwareMap) {
        leftFront = new Motor(hardwareMap, RobotParameters.Motors.HardwareMapNames.leftFront);
        leftBack = new Motor(hardwareMap, RobotParameters.Motors.HardwareMapNames.leftBack);
        rightFront = new Motor(hardwareMap, RobotParameters.Motors.HardwareMapNames.rightFront);
        rightBack = new Motor(hardwareMap, RobotParameters.Motors.HardwareMapNames.rightBack);
        leftIntakeSlide =  new Motor(hardwareMap, RobotParameters.Motors.HardwareMapNames.leftIntakeSlide);
        rightIntakeSlide =  new Motor(hardwareMap, RobotParameters.Motors.HardwareMapNames.rightIntakeSlide);
        leftOuttakeSlide =  new Motor(hardwareMap, RobotParameters.Motors.HardwareMapNames.leftOuttakeSlide);
        rightOuttakeSlide =  new Motor(hardwareMap, RobotParameters.Motors.HardwareMapNames.rightOuttakeSlide);

        // Set motor directions
        leftFront.setInverted(RobotParameters.Motors.Reversed.leftFront);
        leftBack.setInverted(RobotParameters.Motors.Reversed.leftBack);
        rightFront.setInverted(RobotParameters.Motors.Reversed.rightFront);
        rightBack.setInverted(RobotParameters.Motors.Reversed.rightBack);
        leftOuttakeSlide.setInverted(true);
        rightOuttakeSlide.setInverted(false);
        leftIntakeSlide.setInverted(true);
        rightIntakeSlide.setInverted(false);

        // Set braking behaviour
        leftFront.setZeroPowerBehavior(RobotParameters.Motors.zeroPowerBehaviour);
        leftBack.setZeroPowerBehavior(RobotParameters.Motors.zeroPowerBehaviour);
        rightFront.setZeroPowerBehavior(RobotParameters.Motors.zeroPowerBehaviour);
        rightBack.setZeroPowerBehavior(RobotParameters.Motors.zeroPowerBehaviour);
        leftIntakeSlide.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftOuttakeSlide.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        rightIntakeSlide.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightOuttakeSlide.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        rightOuttakeSlide.resetEncoder();
        leftOuttakeSlide.resetEncoder();
        leftIntakeSlide.resetEncoder();
        rightIntakeSlide.resetEncoder();

        // Initialize Tracking Classes
        powers = new MotorPowers();
    }

    public class MotorPowers {
        public double leftFront = 0.0;
        public double leftBack = 0.0;
        public double rightFront = 0.0;
        public double rightBack = 0.0;
        public double leftOuttakeSlide = 0.0;
        public double rightOuttakeSlide = 0.0;
        public double leftIntakeSlide = 0.0;
        public double rightIntakeSlide = 0.0;
    }

    public void setPowers() {
        if (Math.abs(powers.leftFront) > 0.05) { leftFront.set(powers.leftFront); } else { leftFront.set(0.0); }
        if (Math.abs(powers.rightFront) > 0.05) { rightFront.set(powers.rightFront); } else { rightFront.set(0.0); }
        if (Math.abs(powers.leftBack) > 0.05) { leftBack.set(powers.leftBack); } else { leftBack.set(0.0); }
        if (Math.abs(powers.rightBack) > 0.05) { rightBack.set(powers.rightBack); } else { rightBack.set(0.0); }
        if (Math.abs(powers.leftOuttakeSlide) > 0.05) { leftOuttakeSlide.set(powers.leftOuttakeSlide); } else { leftOuttakeSlide.set(0.0); }
        if (Math.abs(powers.rightOuttakeSlide) > 0.05) { rightOuttakeSlide.set(powers.rightOuttakeSlide); } else { rightOuttakeSlide.set(0.0); }
        if (Math.abs(powers.leftIntakeSlide) > 0.05) { leftIntakeSlide.set(powers.leftIntakeSlide); } else { leftIntakeSlide.set(0.0); }
        if (Math.abs(powers.rightIntakeSlide) > 0.05) { rightIntakeSlide.set(powers.rightIntakeSlide); } else { rightIntakeSlide.set(0.0); }
    }

    public void stopMotors() {
        powers.leftBack = 0.0;
        powers.rightBack = 0.0;
        powers.leftFront = 0.0;
        powers.rightFront = 0.0;
        setPowers();
    }
}
