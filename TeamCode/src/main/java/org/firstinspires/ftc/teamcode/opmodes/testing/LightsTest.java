package org.firstinspires.ftc.teamcode.opmodes.testing;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.Robot;

public class LightsTest extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);

        if (isStopRequested()) {return;}
        waitForStart();

        robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);
        while (opModeIsActive()) {
            continue;
        }
    }
}
