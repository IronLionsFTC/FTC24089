package org.firstinspires.ftc.teamcode.core.control;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.core.control.ButtonEnum;

public class Button {
    private final ButtonEnum button;
    private int pressedFrames = 0;
    private int previousFrame = 0;

    public Button(ButtonEnum b) { button = b; }

    /// Update frame tracker
    public void update(GamepadEx gamepad) {
        previousFrame = pressedFrames;
        boolean currentlyPressed = gamepad.getButton(button.ftclib_button());
        pressedFrames = currentlyPressed ? pressedFrames + 1 : 0;
    }

    /// @return Whether the button is currently pressed
    public boolean isPressed() { return pressedFrames > 0; }
    /// Rising edge detector
    /// @return Whether the button was just pressed
    public boolean press() { return pressedFrames == 1 && previousFrame == 0; }
    /// Falling edge detector
    /// @return Whether the button was just released
    public boolean release() { return pressedFrames == 0 && previousFrame > 0; }

    /// @return How many loops the button has been pressed for
    public int framesPressed() { return pressedFrames; }
}
