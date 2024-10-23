package org.firstinspires.ftc.teamcode.opmodes;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.params.RobotParameters;

@TeleOp
public class TuneIntakePID extends LinearOpMode
{
    @Config
    public static class ArmTargetPosition {
        public static double T = 0.0;
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        Servo arm = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.armServo);

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
            arm.setPosition(ArmTargetPosition.T);
        }
    }
}