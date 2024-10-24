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
            public static final boolean left = false;
            public static final boolean right = false;
            public static final boolean sideways = true;
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
        public static final double armDown = 0.0;
        public static final double armUp = 0.62;
        public static final double armTransfer = 0.13;
        public static final double bucketOpen = 0.1; // Parallel
        public static final double bucketTransfer = 0.15;
        public static final double bucketClosed = 0.27; // Gripping
        public static final double intakeFolded = 0.0;
        public static final double intakeDown = 0.47; // Just above tiles
    }

    public static final class SlideBounds {
        public static final double outtakeDown = 0.0;
        public static final double outtakeUp = 2950.0;
        public static final double intakeIn = 0.0;
        public static final double intakeClearance = 70.0; // Allow outtake to freely move without clipping
        public static final double intakeExtended = 100.0;
        public static final double intakeTransfer = 42.0;  // Position where transfer is most consistent
    }

    public static final class Thresholds {
        public static final double intakeSamplePresent = 20.0; // mm where sample is present  (if less than)
        public static final double intakeClearanceForOuttakeMovement = 60.0; // When to start moving outtake
        public static final double outtakeMinimumHeightToNotWorryAboutIntake = 800.0;
        public static final double outtakeHeightToRetractIntakeUpper = 900.0; // When outtake reaches this height, retract intake
        public static final double outtakeHeightToRetractIntakeLower = 100.0; //  ^^^
        public static final double colourFilter = 1.0; // Higher the stricter
        public static final double frameThresh = 4.0; // Num loop cycles to be confident that a sample is there
    }

    public static final class PIDConstants {
        // Feedforward
        public static final double ticksInDegree = 8.33;

        // Outtake Slide Settings
        public static final double intakeSlideP = 0.002;
        public static final double intakeSlideI = 0.003;
        public static final double intakeSlideD = 0.0001;
        public static final double intakeSlideF = 0.1;

        // Spinning Intake Speed
        public static final double intakeSpeed = 0.3;
        public static final double reverseIntakeSpeed = 0.5;

        // Yaw Correction Settings
        public static final double yawP = 0.03;
        public static final double yawI = 0.05;
        public static final double yawD = 0.001;
    }
}
