package org.firstinspires.ftc.teamcode.core.control;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.core.control.Button;

public class Controller {
    public GamepadEx gamepad;
    // Main buttons
    public Button Y = new Button(ButtonEnum.Y);
    public Button X = new Button(ButtonEnum.X);
    public Button B = new Button(ButtonEnum.B);
    public Button A = new Button(ButtonEnum.A);
    // D-PAD
    public Button DU = new Button(ButtonEnum.U);
    public Button DD = new Button(ButtonEnum.D);
    public Button DL = new Button(ButtonEnum.L);
    public Button DR = new Button(ButtonEnum.R);
    // Bumpers (buttons above triggers)
    public Button LB = new Button(ButtonEnum.LB);
    public Button RB = new Button(ButtonEnum.RB);
    // Special buttons
    public Button START = new Button(ButtonEnum.START);
    public Button BACK = new Button(ButtonEnum.BACK);

    public Controller(GamepadEx g) { this.gamepad = g; }
    public Controller(Gamepad g) { this.gamepad = new GamepadEx(g); }


    /// Update all buttons' frame trackers.
    /// IMPORTANT! Make sure that this is called before checking buttons.
    /// Not doing so may cause stale input!
    public void update() {
        // Main buttons
        this.X.update(gamepad);
        this.Y.update(gamepad);
        this.A.update(gamepad);
        this.B.update(gamepad);
        // D-Pad
        this.DU.update(gamepad);
        this.DD.update(gamepad);
        this.DL.update(gamepad);
        this.DR.update(gamepad);
        // Bumpers (buttons above triggers)
        this.LB.update(gamepad);
        this.RB.update(gamepad);
        // Special buttons
        this.START.update(gamepad);
        this.BACK.update(gamepad);
    }

    public class Stick {
        // Y-axis of joysticks is inverted, so flip them here
        public double LX() { return gamepad.getLeftX(); }
        public double LY() { return gamepad.getLeftY() * -1; }
        public double RX() { return gamepad.getRightX(); }
        public double RY() { return gamepad.getRightY() * -1; }
    } public Stick stick = new Stick();

    public double LT() { return gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER); }
    public double RT() { return gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER); }
}
