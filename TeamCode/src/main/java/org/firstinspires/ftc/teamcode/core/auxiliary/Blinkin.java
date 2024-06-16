package org.firstinspires.ftc.teamcode.core.auxiliary;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Blinkin {
    private RevBlinkinLedDriver leftLights, rightLights;
    private RevBlinkinLedDriver.BlinkinPattern rightPattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
    private RevBlinkinLedDriver.BlinkinPattern leftPattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;

    public Blinkin(HardwareMap hardwareMap) {
        leftLights = hardwareMap.get(RevBlinkinLedDriver.class, "leftLights");
        rightLights = hardwareMap.get(RevBlinkinLedDriver.class, "rightLights");
    }

    public void reset(LinearOpMode opmode) {
        opmode.resetRuntime();
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern _pattern) {
        leftPattern = _pattern;
        rightPattern = _pattern;
        leftLights.setPattern(leftPattern);
        rightLights.setPattern(rightPattern);
    }

    public void setRightLights(RevBlinkinLedDriver.BlinkinPattern _pattern) {
        rightPattern = _pattern;
        rightLights.setPattern(rightPattern);
    }

    public void setLeftLights(RevBlinkinLedDriver.BlinkinPattern _pattern) {
        leftPattern = _pattern;
        leftLights.setPattern(leftPattern);
    }
}
