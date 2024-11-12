package org.firstinspires.ftc.teamcode.core.params;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

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
            public static final String latchServo = "latchServo";

            // Servo Motors
            public static final String intakeLiftServo = "intakeLiftServo"; // Port c0
            public static final String intakeYawServo = "intakeYawServo"; // Port c1
            public static final String intakeClawServo = "intakeClawServo"; // Port c2
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

    public static final class Odometry {
        public static final class HardwareMapNames {
            public static final String left = RobotParameters.Motors.HardwareMapNames.rightBack;
            public static final String right = RobotParameters.Motors.HardwareMapNames.leftFront;
            public static final String sideways = RobotParameters.Motors.HardwareMapNames.rightFront;
        }
        public static final class Reversed {
            public static final boolean left = true;
            public static final boolean right = true;
            public static final boolean sideways = true;
        }
        public static final class CenterOffset_mm {
            // See diagram in LOCALIZATION.md
            public static final double leftx = 105;
            public static final double lefty = 180;
            public static final double rightx = 105;
            public static final double righty = -180;
            public static final double sidex = -40;
            public static final double sidey = 10;
        }
        public static final class CenterOffset_in {
            public static final double leftx = RobotParameters.Odometry.CenterOffset_mm.leftx / 25.4;
            public static final double lefty = RobotParameters.Odometry.CenterOffset_mm.lefty  / 25.4;
            public static final double rightx = RobotParameters.Odometry.CenterOffset_mm.rightx  / 25.4;
            public static final double righty = RobotParameters.Odometry.CenterOffset_mm.righty  / 25.4;
            public static final double sidex = RobotParameters.Odometry.CenterOffset_mm.sidex  / 25.4;
            public static final double sidey = RobotParameters.Odometry.CenterOffset_mm.sidey  / 25.4;
        }
    }

    public static final class Movement {
        public static final double strafeCorrection = 1.1;
        public static final double speed = 1.0;
    }

    public static final class IMU {
        public static final RevHubOrientationOnRobot hubOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        public static final String hardwareMapName = "imu";
        public static final double PIDcorrectionThreshold = 5.0;
        public static final double correctionMultiplier = 1.0;
        public static final double rotationMinimumThreshold = 0.0;
    }

    public static final class ServoBounds {
        public static final double armDown = 0.03;
        public static final double armUp = 0.57;
        public static final double armWait = 0.37;
        public static final double armTransfer = 0.13;

        public static final double clawClosed = 0.0;
        public static final double clawOpen = 0.4;

        // NEW
        public static final double intakeFolded = 0.0;
        public static final double intakeDown = 1.0;
        public static final double intakeYawZero = 0.63;

        public static final double latchClosed = 0.13;
        public static final double latchOpened = 0.0;
    }

    public static final class SlideBounds {
        public static final double outtakeDown = 0.02;
        public static final double outtakeUp = 2950.0;
        public static final double intakeIn = 0.0;
        public static final double intakeClearance = 70.0; // Allow outtake to freely move without clipping
        public static final double intakeExtended = 90.0;
    }

    public static final class Thresholds {
        public static final double intakeSamplePresent = 20.0; // mm where sample is present  (if less than)
        public static final double intakeClearanceForOuttakeMovement = 60.0; // When to start moving outtake
        public static final double outtakeMinimumHeightToNotWorryAboutIntake = 500.0;
        public static final double outtakeHeightToRetractIntakeUpper = 500.0; // When outtake reaches this height, retract intake
        public static final double outtakeHeightToRetractIntakeLower = 150.0; //  ^^^
        public static final double colourFilter = 0.8; // Higher the stricter
        public static final double frameThresh = 4.0; // Num loop cycles to be confident that a sample is there

        public static final int CVSmoothing = 1; // Num frames to smooth values over to reduce jitter.
    }

    public static final class PIDConstants {
        // Feedforward
        public static final double ticksInDegree = 8.33;

        // Outtake Slide Settings
        public static final double outtakeSlideP = 0.002;
        public static final double outtakeSlideI = 0.003;
        public static final double outtakeSlideD = 0.0001;
        public static final double outtakeSlideF = 0.1;

        // Spinning Intake Speed
        public static final double intakeSpeed = 0.2;
        public static final double reverseIntakeSpeed = 0.2;

        // Yaw Correction Settings
        public static final double yawP = 0.015;
        public static final double yawI = 0.05;
        public static final double yawD = 0.001;
    }
}
