package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.core.params.RobotParameters;
import org.firstinspires.ftc.teamcode.core.state.Team;

@TeleOp(name="EncoderDirectionDebug", group="Debug")
public class EncoderDirectionDebug extends LinearOpMode
{
    Robot robot;
    GamepadEx gamepad;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new Robot(hardwareMap, telemetry, Team.Red);
        gamepad = new GamepadEx(gamepad1);

        MotorEx encoderLeft = new MotorEx(hardwareMap, RobotParameters.Odometry.HardwareMapNames.left);
        MotorEx encoderRight = new MotorEx(hardwareMap, RobotParameters.Odometry.HardwareMapNames.right);
        MotorEx encoderSideways = new MotorEx(hardwareMap, RobotParameters.Odometry.HardwareMapNames.sideways);

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
        while (opModeIsActive()){
            telemetry.addData("Left", encoderLeft.getCurrentPosition());
            telemetry.addData("Right", encoderRight.getCurrentPosition());
            telemetry.addData("Sideways", encoderSideways.getCurrentPosition());
            telemetry.update();
        }
    }
}
