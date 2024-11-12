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
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Encoder;

@TeleOp(name="EncoderDirectionDebug", group="Debug")
public class EncoderDirectionDebug extends LinearOpMode
{
    Robot robot;
    GamepadEx gamepad;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new Robot(hardwareMap, telemetry, gamepad1, Team.Red);
        gamepad = new GamepadEx(gamepad1);

        MotorEx encoderLeft = new MotorEx(hardwareMap, RobotParameters.Odometry.HardwareMapNames.left);
        MotorEx encoderRight = new MotorEx(hardwareMap, RobotParameters.Odometry.HardwareMapNames.right);
        MotorEx encoderSideways = new MotorEx(hardwareMap, RobotParameters.Odometry.HardwareMapNames.sideways);
        int leftRev = RobotParameters.Odometry.Reversed.left ? -1 : 1;
        int rightRev = RobotParameters.Odometry.Reversed.right ? -1 : 1;
        int sidewaysRev = RobotParameters.Odometry.Reversed.sideways ? -1 : 1;

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
            telemetry.addData("Left", encoderLeft.getCurrentPosition() * leftRev);
            telemetry.addData("Right", encoderRight.getCurrentPosition() * rightRev);
            telemetry.addData("Sideways", encoderSideways.getCurrentPosition() * sidewaysRev);
            telemetry.update();
        }
    }
}
