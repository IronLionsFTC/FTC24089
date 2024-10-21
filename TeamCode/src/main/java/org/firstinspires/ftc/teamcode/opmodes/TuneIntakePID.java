package org.firstinspires.ftc.teamcode.opmodes;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.core.params.RobotParameters;

@TeleOp
public class TuneIntakePID extends LinearOpMode
{
    @Config
    public static class PID {
        public static double P = 0.004;
        public static double I = 0.01;
        public static double D = 0.0;
        public static double F = 0.0;
        public static double T = 0.0; // Target
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        CRServo arm = new CRServo(hardwareMap, RobotParameters.Motors.HardwareMapNames.armServo);
        AnalogInput armcoder = hardwareMap.get(AnalogInput.class, "armEncoder");
        PIDController controller = new PIDController(PID.P, PID.I, PID.D);

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
            double voltage = armcoder.getVoltage();
            double position = (252.0 - (voltage / 3.33) * 360.0) / 235.0 * 360.0;
            controller.setPID(PID.P, PID.I, PID.D);
            double response = controller.calculate(position, PID.T);
            double power = Math.cos(Math.toRadians(PID.T / RobotParameters.PIDConstants.ticksInDegree)) * PID.F;
            if (position > 20.0 && position < 280.0) {
                power = 0.0;
            }
            arm.set(power + response);
            telemetry.addData("pos", position);
            telemetry.addData("target", PID.T);
            telemetry.update();
            // bottom : 0
            // top : 325
            // transfer: 30
        }
    }
}