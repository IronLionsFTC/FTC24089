package org.firstinspires.ftc.teamcode.core;


import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.core.params.Controls;
import org.firstinspires.ftc.teamcode.core.params.RobotParameters;

public class Controller {
    public static class StickInputsRaw {
        public static double LX(GamepadEx gamepad) {
            return gamepad.getLeftX();
        }
        public static double LY(GamepadEx gamepad) {
            return -gamepad.getLeftY();
        }
        public static double RX(GamepadEx gamepad) {
            return gamepad.getRightX();
        }
        public static double RY(GamepadEx gamepad) {
            return -gamepad.getRightY();
        }
    }

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
