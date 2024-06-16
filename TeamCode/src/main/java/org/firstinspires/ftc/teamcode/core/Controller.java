package org.firstinspires.ftc.teamcode.core;


import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.core.params.Controls;
import org.firstinspires.ftc.teamcode.core.params.RobotParameters;

public class Controller {
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
        public static double LT(GamepadEx gamepad) { return gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER); }
        public static double RT(GamepadEx gamepad) { return gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER); }
    }

    public double right_trigger(GamepadEx gamepad) { return Controls.Trigger.RT.getValue(gamepad);}
    public double left_trigger(GamepadEx gamepad) { return Controls.Trigger.LT.getValue(gamepad); }
    public double movement_x(GamepadEx gamepad) {
        return Controls.movementX.get(gamepad);
    }
    public double movement_y(GamepadEx gamepad) {
        return Controls.movementY.get(gamepad);
    }
    public double rotation(GamepadEx gamepad) {
        return Controls.rotationAxis.get(gamepad);
    }
}
