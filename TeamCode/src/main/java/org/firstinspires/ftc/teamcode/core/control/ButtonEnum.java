package org.firstinspires.ftc.teamcode.core.control;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public enum ButtonEnum {
    A(GamepadKeys.Button.A),
    B(GamepadKeys.Button.B),
    X(GamepadKeys.Button.X),
    Y(GamepadKeys.Button.Y),
    U(GamepadKeys.Button.DPAD_UP),
    D(GamepadKeys.Button.DPAD_DOWN),
    R(GamepadKeys.Button.DPAD_RIGHT),
    L(GamepadKeys.Button.DPAD_LEFT),
    LB(GamepadKeys.Button.LEFT_BUMPER),
    RB(GamepadKeys.Button.RIGHT_BUMPER),
    START(GamepadKeys.Button.START),
    BACK(GamepadKeys.Button.BACK);

    private final GamepadKeys.Button button;

    ButtonEnum(GamepadKeys.Button b) { button = b; }

    public GamepadKeys.Button ftclib_button() { return button; }
}
