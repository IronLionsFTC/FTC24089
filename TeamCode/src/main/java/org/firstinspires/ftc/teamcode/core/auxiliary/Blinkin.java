package org.firstinspires.ftc.teamcode.core.auxiliary;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Blinkin {
    private RevBlinkinLedDriver leftLights, rightLights;
    private boolean blinkinTimer = false;
    private int blinkinDelay = 2000;

    private RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE;

    public Blinkin(HardwareMap hardwareMap) {
        leftLights = hardwareMap.get(RevBlinkinLedDriver.class, "leftLights");
        rightLights = hardwareMap.get(RevBlinkinLedDriver.class, "rightLights");
    }

    public void reset(LinearOpMode opmode) {
        blinkinTimer = true;
        opmode.resetRuntime();
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern _pattern) {
        pattern = _pattern;
    }
}
