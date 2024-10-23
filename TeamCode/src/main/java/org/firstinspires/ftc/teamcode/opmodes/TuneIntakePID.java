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
        public static double intakeT = 0.0; // 0.49
        public static double armT = 0.0;    // 0.64 top, 0.0 bottom, 0.09 transfer
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        Servo arm = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.armServo);
        Servo intakeA = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.leftIntakeLiftServo);
        Servo intakeB = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.rightIntakeLiftServo);
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
            arm.setPosition(ArmTargetPosition.armT);
            intakeA.setPosition(1.0 - ArmTargetPosition.intakeT);
            intakeB.setPosition(ArmTargetPosition.intakeT);
        }
    }
}