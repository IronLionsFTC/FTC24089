package org.firstinspires.ftc.teamcode.core.params;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Encoder;

public class RobotParameters {
    private RobotParameters(){} // Hide the constructor
    public static final double radius = 0.96;
    public static final double slideWeightCompensation = 0.6; // Do not go above ~0.5.
    public static final class Motors {
        public static final Motor.ZeroPowerBehavior zeroPowerBehaviour = Motor.ZeroPowerBehavior.BRAKE;

        public static final class HardwareMapNames {
            public static final String leftFront = "leftFront";
            public static final String leftBack = "leftBack";
            public static final String rightFront = "rightFront";
            public static final String rightBack = "rightBack";
            public static final String leftIntakeSlide = "leftIntakeSlide";
            public static final String rightIntakeSlide = "rightIntakeSlide";
            public static final String leftOuttakeSlide = "leftOuttakeSlide";
            public static final String rightOuttakeSlide = "rightOuttakeSlide";

            // Servo Motors
            public static final String bucketServo = "BucketServo";  // Port 1 ?
            public static final String armServo = "ArmServo";        // Port 0 ?
            public static final String leftIntakeLiftServo = "leftIntakeLiftServo"; // Port e0
            public static final String rightIntakeLiftServo = "rightIntakeLiftServo"; // Port e1

            // CR Servo Motors
            public static final String intakeServoA = "intakeServoA"; // Port 2
            public static final String intakeServoB = "intakeServoB"; // Port 3
        }
        public static final class Reversed {
            public static final boolean leftFront = true;
            public static final boolean leftBack = true;
            public static final boolean rightFront = false;
            public static final boolean rightBack = false;
            public static final boolean leftIntake = true;
            public static final boolean rightIntake = false;
            public static final boolean leftSlide = true;
            public static final boolean rightSlide = false;
        }
    }

    public static final class Odometry { // Last verified 13/6/24 8:21AM
        public static final class HardwareMapNames {
            public static final String left = RobotParameters.Motors.HardwareMapNames.rightBack;
            public static final String right = RobotParameters.Motors.HardwareMapNames.leftFront;
            public static final String sideways = RobotParameters.Motors.HardwareMapNames.rightFront;
        }
        public static final class Reversed {
            public static final boolean left = true;
            public static final boolean right = false;
            public static final boolean sideways = false;
        }
    }

    public static final class Movement {
        public static final double strafeCorrection = 1.1;
        public static final double speed = 1.0;
    }

    public static final class IMU {
        public static final RevHubOrientationOnRobot hubOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        );

        public static final String hardwareMapName = "imu";
        public static final double PIDcorrectionThreshold = 5.0;
        public static final double correctionMultiplier = 1.0;
        public static final double rotationMinimumThreshold = 0.0;
    }

    public static final class ServoBounds {
        public static final double armServoLower = 0.0;
        public static final double armServoUpper = 1.0;
        public static final double bucketServoLower = 0.7;
        public static final double bucketServoUpper = 0.0;
    }

    public static final class SlideBounds {
        public static final double outtake = 2500;
    }
}
