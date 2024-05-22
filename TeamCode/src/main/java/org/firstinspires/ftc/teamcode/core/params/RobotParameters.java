package org.firstinspires.ftc.teamcode.core.params;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class RobotParameters {
    private RobotParameters(){} // Hide the constructor

    public static final class Motors {
        public static final Motor.ZeroPowerBehavior zeroPowerBehaviour = Motor.ZeroPowerBehavior.BRAKE;

        public static final class HardwareMapNames {
            public static final String leftFront = "leftFront";
            public static final String leftBack = "leftBack";
            public static final String rightFront = "rightFront";
            public static final String rightBack = "rightBack";
        }
        public static final class Reversed {
            public static final boolean leftFront = false;
            public static final boolean leftBack = false;
            public static final boolean rightFront = false;
            public static final boolean rightBack = false;
        }
    }

    public static final class Movement {
        public static final double strafeCorrection = 1.1;
        public static final double speed = 0.5;
    }

    public static final class IMU {
        public static final RevHubOrientationOnRobot hubOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        );

        public static final String hardwareMapName = "imu";

        public static final double PIDcorrectionThreshold = 5.0;
        public static final double correctionMultiplier = 1.0;
        public static final double rotationSpeedMultiplier = 1.0;
        public static final double rotationMinimumThreshold = 0.01;
    }
}
