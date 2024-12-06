package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class CachedServo {
    Servo hardware;
    double positionValue;

    public CachedServo(HardwareMap hardwareMap, String name) {
        hardware = hardwareMap.get(Servo.class, name);
        positionValue = -1.0;
    }

    public void setPosition(double position) {
        if (position != positionValue) {
            positionValue = position;
            hardware.setPosition(positionValue);
        }
    }

    public void kill() {
        hardware.getController().pwmDisable();
    }

    public void revive() {
        hardware.getController().pwmEnable();
    }
}
