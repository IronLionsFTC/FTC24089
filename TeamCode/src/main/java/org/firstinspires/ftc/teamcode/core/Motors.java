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
    public Motor leftIntake;
    public Motor rightIntake;

    // Outake motors
    public Motor leftSlide;
    public Motor rightSlide;

    // Motor Powers
    public MotorPowers powers;
    public SlidePositions slidePositions;

    public Motors(HardwareMap hardwareMap) {
        leftFront = new Motor(hardwareMap, RobotParameters.Motors.HardwareMapNames.leftFront);
        leftBack = new Motor(hardwareMap, RobotParameters.Motors.HardwareMapNames.leftBack);
        rightFront = new Motor(hardwareMap, RobotParameters.Motors.HardwareMapNames.rightFront);
        rightBack = new Motor(hardwareMap, RobotParameters.Motors.HardwareMapNames.rightBack);

        // Set motor directions
        leftFront.setInverted(RobotParameters.Motors.Reversed.leftFront);
        leftBack.setInverted(RobotParameters.Motors.Reversed.leftBack);
        rightFront.setInverted(RobotParameters.Motors.Reversed.rightFront);
        rightBack.setInverted(RobotParameters.Motors.Reversed.rightBack);

        // Set braking behaviour
        leftFront.setZeroPowerBehavior(RobotParameters.Motors.zeroPowerBehaviour);
        leftBack.setZeroPowerBehavior(RobotParameters.Motors.zeroPowerBehaviour);
        rightFront.setZeroPowerBehavior(RobotParameters.Motors.zeroPowerBehaviour);
        rightBack.setZeroPowerBehavior(RobotParameters.Motors.zeroPowerBehaviour);

        // Initialize Tracking Classes
        powers = new MotorPowers();
        slidePositions = new SlidePositions();
    }

    public class MotorPowers {
        public double leftFront = 0.0;
        public double leftBack = 0.0;
        public double rightFront = 0.0;
        public double rightBack = 0.0;
        public double leftIntake = 0.0;
        public double rightIntake = 0.0;
        public double leftSlide = 0.0;
        public double rightSlide = 0.0;
    }

    public class SlidePositions {
        public double outTake = 0.0;
        public double outTakeLastError = 0.0;
        public double inTake = 0.0;
        public double inTakeLastError = 0.0;
    }

    public void setPowers() {
        leftFront.set(powers.leftFront);
        rightFront.set(powers.rightFront);
        leftBack.set(powers.leftBack);
        rightBack.set(powers.rightBack);
    }

    public void stopMotors() {
        powers.leftBack = 0.0;
        powers.rightBack = 0.0;
        powers.leftFront = 0.0;
        powers.rightFront = 0.0;
        setPowers();
    }
}
