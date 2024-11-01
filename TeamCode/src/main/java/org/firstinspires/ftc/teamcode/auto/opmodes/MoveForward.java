package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.core.state.Team;
import org.firstinspires.ftc.teamcode.ironauto.IronAuto;

@Autonomous(name = "MoveForwardTest", group = "Testing")
public class MoveForward extends LinearOpMode {
    private IronAuto auto;
    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException
    {
        this.robot = new Robot(hardwareMap, telemetry, Team.Blue);
        this.auto = new IronAuto(this.robot);

        //////////////////////////////////////////
        // Runs when the init button is pressed //
        //////////////////////////////////////////

        if (isStopRequested()) return;
        waitForStart();

        //////////////////////////////////////////
        // Runs when the play button is pressed //
        //////////////////////////////////////////

        // Main loop
        while (opModeIsActive()){
            this.auto.move_forward(30.0);
            this.auto.move_forward(-30.0);
        }
    }
}
