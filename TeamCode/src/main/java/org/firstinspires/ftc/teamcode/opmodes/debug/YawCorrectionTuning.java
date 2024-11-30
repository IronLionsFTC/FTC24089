package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.core.state.Team;

@TeleOp
public class YawCorrectionTuning extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        //////////////////////////////////////////
        // Runs when the init button is pressed //
        //////////////////////////////////////////

        Robot robot = new Robot(hardwareMap, telemetry, null, null, Team.Blue);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (isStopRequested()) return;
        waitForStart();

        //////////////////////////////////////////
        // Runs when the play button is pressed //
        //////////////////////////////////////////

        // Main loop
        while (opModeIsActive()) {
            robot.orientation.update();
            telemetry.addData("YAW", robot.orientation.getYaw());
            telemetry.update();
        }
    }
}
