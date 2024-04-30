package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Custom
import org.firstinspires.ftc.teamcode.core.Robot;

@TeleOp
public class BaseOpMode extends LinearOpMode
{
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new Robot(hardwareMap, telemetry);

        // Runs when "init" is pressed
        robot.drive.motors.init();
        robot.drive.robotIMU.initIMU();

        if (isStopRequested()) return;
        waitForStart();
        //////////////////////////////////////////
        // Runs when the play button is pressed //
        //////////////////////////////////////////

        // Main loop
        while (opModeIsActive()){
            robot.drive.movement.moveTele(gamepad1);
        }
    }
}