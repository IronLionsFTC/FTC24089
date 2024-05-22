package org.firstinspires.ftc.teamcode.core.params;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public final class Controls {
    public static enum JoystickAxis {
        LX,
        LY,
        RX,
        RY;

        public double get(GamepadEx gamepad) {
            switch (this) {
                case LX:
                    return gamepad.getLeftX();
                case LY:
                    return -gamepad.getLeftY();
                case RX:
                    return gamepad.getRightX();
                case RY:
                    return -gamepad.getRightY();
            }
            return 0.0;
        }
    }
    public static enum Button {
        A(GamepadKeys.Button.A),
        B(GamepadKeys.Button.B),
        X(GamepadKeys.Button.X),
        Y(GamepadKeys.Button.Y);

        private final GamepadKeys.Button button;

        Button(GamepadKeys.Button b) {
            button = b;
        }

        public boolean isPressed(GamepadEx gamepad) {
            return gamepad.getButton(button);
        }
    }

    // Movement
    public static final JoystickAxis movementX = JoystickAxis.LX;
    public static final JoystickAxis movementY = JoystickAxis.LY;
    // Rotation
    public static final JoystickAxis rotationAxis = JoystickAxis.RX;
    // Yaw correction
    public static final Button resetYaw = Button.A;
    public static final Button driverOverride = Button.B;
}
