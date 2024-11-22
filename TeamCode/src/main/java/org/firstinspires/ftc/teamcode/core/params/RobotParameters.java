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

            public static final String outtakeClawServo = "outtakeClawServo";
            public static final String leftArmServo = "leftArmServo";
            public static final String rightArmServo = "rightArmServo";
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
            public static final String sideways = RobotParameters.Motors.HardwareMapNames.leftBack;
        }
        public static final class Reversed {
            public static final boolean left = true;
            public static final boolean right = true;
            public static final boolean sideways = false;
        }
        public static final class CenterOffset_mm {
            // See diagram in LOCALIZATION.md
            // LEFT: 153mm to the LEFT, 105mm forward
            // RIGHT: 153mm to the RIGHT, 105mm forward
            // SIDEWAYS [TEMPORARY, WE MIGHT CHANGE THIS]: 110mm forward, 26mm RIGHT
            public static final double leftx = 105;
            public static final double lefty = 153;
            public static final double rightx = 105;
            public static final double righty = -153;
            public static final double sidex = 110;
            public static final double sidey = -26;
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
        public static final double preciseMovementSpeed = 0.4;
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
        public static final double armDown = 0.0;
        public static final double armUp = 0.62;

        public static final double clawClosed = 0.0;
        public static final double clawOpen = 0.2;
        public static final double clawWideOpen = 0.4;

        public static final double intakeFolded = 1.0;
        public static final double intakeDown = 0.0;
        public static final double intakeYawZero = 0.64;

        public static final double intakeYawFlipped = 0.09;
        public static final double intakeGrabOffWall = 0.0;

        public static final double latchOpen = 0.0;
        public static final double latchShut = 0.4;
    }

    public static final class SlideBounds {
        public static final double outtakeDown = 0.02;
        public static final double outtakeUp = 900.0;
        public static final double intakeIn = 0.0;
        public static final double intakeExtended = 150;

        public static final double outtakeBelowSpecimenBar = 180.0;
        public static final double outtakeOnSpecimenBar = 550.0;
    }

    public static final class Thresholds {
        public static final int CVSmoothing = 1; // Num frames to smooth values over to reduce jitter.
    }

    public static final class PIDConstants {
        // Feedforward
        public static final double ticksInDegree = 8.33;

        // Outtake Slide Settings
        public static final double outtakeSlideP = 0.004;
        public static final double outtakeSlideI = 0.0;
        public static final double outtakeSlideD = 0.0;
        public static final double outtakeSlideF = 0.15;

        // Yaw Correction Settings
        public static final double yawP = 0.02;
        public static final double yawI = 0.0;
        public static final double yawD = 0.0;
    }
}
