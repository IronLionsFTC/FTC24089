package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

// Custom
import org.firstinspires.ftc.teamcode.core.Robot;

@TeleOp
public class BaseOpMode extends LinearOpMode
{
    Robot robot;
    GamepadEx gamepad;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new Robot(hardwareMap, telemetry);
        gamepad = new GamepadEx(gamepad1);

        //////////////////////////////////////////
        // Runs when the init button is pressed //
        //////////////////////////////////////////

        // code here

        // Do not touch
        if (isStopRequested()) return;
        waitForStart();

        //////////////////////////////////////////
        // Runs when the play button is pressed //
        //////////////////////////////////////////

        // Main loop
        while (opModeIsActive()){
            robot.drivetrain.moveTele(gamepad);
        }
    }
}