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
        Y(GamepadKeys.Button.Y),
        U(GamepadKeys.Button.DPAD_UP),
        D(GamepadKeys.Button.DPAD_DOWN),
        R(GamepadKeys.Button.DPAD_RIGHT),
        L(GamepadKeys.Button.DPAD_LEFT);

        private final GamepadKeys.Button button;

        Button(GamepadKeys.Button b) {
            button = b;
        }

        public boolean isPressed(GamepadEx gamepad) {
            return gamepad.getButton(button);
        }
    }

    public static enum Trigger {
        RT(GamepadKeys.Trigger.RIGHT_TRIGGER),
        LT(GamepadKeys.Trigger.LEFT_TRIGGER);

        private final GamepadKeys.Trigger trigger;

        Trigger(GamepadKeys.Trigger t) { trigger = t; }

        public double getValue(GamepadEx gamepad) { return gamepad.getTrigger(trigger); }
        public boolean isPressed(GamepadEx gamepad) { return gamepad.getTrigger(trigger) > 0.75; }
    }

    // Movement
    public static final JoystickAxis movementX = JoystickAxis.LX;
    public static final JoystickAxis movementY = JoystickAxis.LY;
    // Rotation
    public static final JoystickAxis rotationPitchAxis = JoystickAxis.RX;
    public static final JoystickAxis rotationYawAxis = JoystickAxis.RY;
}
