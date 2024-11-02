package org.firstinspires.ftc.teamcode.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.core.state.Team;

@TeleOp
public class RedTeamTeleop extends LinearOpMode
{
    Robot robot;
    GamepadEx gamepad;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new Robot(hardwareMap, telemetry, Team.Red);
        gamepad = new GamepadEx(gamepad1);

        //////////////////////////////////////////
        // Runs when the init button is pressed //
        //////////////////////////////////////////

        if (isStopRequested()) return;
        waitForStart();

        //////////////////////////////////////////
        // Runs when the play button is pressed //
        //////////////////////////////////////////

        robot.blockingRetractAndZeroBothSlides();

        // Main loop
        while (opModeIsActive()){
            if (robot.drivetrain.drive(gamepad)) {
                terminateOpModeNow();
            }
        }
    }
}