package org.firstinspires.ftc.teamcode.opmodes.testing;


import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.core.auxiliary.Blinkin;

public class LightsTest extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);

        robot.drive.motors.init();
        robot.drive.robotIMU.initIMU();

        if (isStopRequested()) {return;}
        waitForStart();

        robot
        while (opModeIsActive()) {

        }
    }
}
