package org.firstinspires.ftc.teamcode.core.control;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.core.params.RobotParameters;

public final class Controls {
    public Controller g1;
    public Controller g2;

    public Controls(Controller g1, Controller g2) {
        this.g1 = g1;
        this.g2 = g2;
    }
    public Controls(GamepadEx g1, GamepadEx g2) {
        this.g1 = new Controller(g1);
        this.g2 = new Controller(g2);
    }
    public Controls(Gamepad g1, Gamepad g2) {
        this.g1 = new Controller(g1);
        this.g2 = new Controller(g2);
    }

    public void update() {
        g1.update();
        g2.update();
    }


    public class Movement {
        // Robot movement
        public double X() { return g1.stick.LX(); }
        public double Y() { return g1.stick.LY(); }
        // Precise movement
        public class Precise {
            double speed = RobotParameters.Movement.preciseMovementSpeed;
            public double X() {
                if (g1.DL.isPressed()) { return -speed; }
                else if (g1.DR.isPressed()) { return speed; }
                else { return 0; }
            }
            public double Y() {
                if (g1.DD.isPressed()) { return -speed; }
                else if (g1.DU.isPressed()) { return speed; }
                else { return 0; }
            }
        } public Precise precise = new Precise();

        // Rotation
        public double yaw() { return g1.stick.RX(); }
        public double pitch() { return g1.stick.RY(); }
        // 45-degree rotation increments
        public boolean CW45() { return g1.RB.press(); }
        public boolean CCW45() { return g1.LB.press(); }

    } public Movement movement = new Movement();

    public class Intake {
        public boolean toggle_state() { return g1.X.press(); }

        public class Claw {
            public double CW_rotation() { return g1.RT(); }
            public double CCW_rotation() { return g1.LT(); }
            public boolean deposit() { return g1.DD.press(); }
        } public Claw claw = new Claw();
    } public Intake intake = new Intake();

    public class Outtake {
        public boolean toggle_state() { return g1.A.press(); }
        public boolean retract() { return g1.X.press(); }
        public boolean reset() { return g1.DR.isPressed(); }
    } public Outtake outtake = new Outtake();

    public boolean util_button_press() { return g1.Y.press(); }
    public boolean use_cv() { return g1.Y.isPressed(); }

    // Emergency overrides
    public boolean RESET() { return g1.B.isPressed(); }
    public boolean EMERGENCY_STOP() { return g1.BACK.isPressed(); }
}
