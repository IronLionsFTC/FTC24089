package org.firstinspires.ftc.teamcode.core;


import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.core.params.Controls;
import org.firstinspires.ftc.teamcode.core.params.RobotParameters;

public class Controller {
    public int yPress = 0;
    public int bPress = 0;
    public int aPress = 0;
    public int xPress = 0;

    public static class StickInputsRaw {
        public static double LX(GamepadEx gamepad) {
            return gamepad.getLeftX();
        }
        public static double LY(GamepadEx gamepad) {
            return gamepad.getLeftY();
        }
        public static double RX(GamepadEx gamepad) {
            return gamepad.getRightX();
        }
        public static double RY(GamepadEx gamepad) {
            return gamepad.getRightY();
        }
        public static double LT(GamepadEx gamepad) {
            return gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER); 
        }
        public static double RT(GamepadEx gamepad) {
            return gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER); 
        }
    }

    public double right_trigger(GamepadEx gamepad) {
        return Controls.Trigger.RT.getValue(gamepad);
    }
    public double left_trigger(GamepadEx gamepad) {
        return Controls.Trigger.LT.getValue(gamepad); 
    }
    public double movement_x(GamepadEx gamepad) {
        return Controls.movementX.get(gamepad);
    }
    public double movement_y(GamepadEx gamepad) {
        return Controls.movementY.get(gamepad);
    }
    public double pitchRotation(GamepadEx gamepad) {
        return Controls.rotationPitchAxis.get(gamepad);
    }
    public double yawRotation(GamepadEx gamepad) {
        return Controls.rotationYawAxis.get(gamepad);
    }
    public void updateKeyTracker(GamepadEx gamepad) {
        if (Controls.Button.Y.isPressed(gamepad)) { yPress += 1; }
        else { yPress = 0; }
        if (Controls.Button.X.isPressed(gamepad)) { xPress += 1; }
        else { xPress = 0; }
        if (Controls.Button.A.isPressed(gamepad)) { aPress += 1; }
        else { aPress = 0; }
        if (Controls.Button.B.isPressed(gamepad)) { bPress += 1; }
        else { bPress = 0; }
    }
}
