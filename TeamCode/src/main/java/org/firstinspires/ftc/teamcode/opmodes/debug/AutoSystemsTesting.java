package org.firstinspires.ftc.teamcode.opmodes.debug;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Controller;
import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.core.Sensors;
import org.firstinspires.ftc.teamcode.core.Servos;
import org.firstinspires.ftc.teamcode.core.state.Team;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@TeleOp
public class AutoSystemsTesting extends LinearOpMode
{
    @Config
    public static class Tune {

    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        Robot robot = new Robot(hardwareMap, telemetry, Team.Blue);
        Controller controller = new Controller();

        //////////////////////////////////////////
        // Runs when the init button is pressed //
        //////////////////////////////////////////

        if (isStopRequested()) return;
        waitForStart();

        //////////////////////////////////////////
        // Runs when the play button is pressed //
        //////////////////////////////////////////
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // Main loop

        GamepadEx gp = new GamepadEx(gamepad1);
        Timer timer = new Timer();

        while (opModeIsActive()) {
            while (controller.xPress != 1) {
                controller.updateKeyTracker(gp);
            }
            terminateOpModeNow();
        }
    }
}
