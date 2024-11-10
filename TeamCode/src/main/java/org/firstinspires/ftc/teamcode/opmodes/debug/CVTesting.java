package org.firstinspires.ftc.teamcode.opmodes.debug;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Motors;
import org.firstinspires.ftc.teamcode.core.Sensors;
import org.firstinspires.ftc.teamcode.core.Servos;
import org.firstinspires.ftc.teamcode.core.Vec2;
import org.firstinspires.ftc.teamcode.core.params.RobotParameters;
import org.firstinspires.ftc.teamcode.core.state.ComputerVision;

@TeleOp
public class CVTesting extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {

        ComputerVision cvTest = new ComputerVision(hardwareMap);
        Servos servos = new Servos(hardwareMap);

        //////////////////////////////////////////
        // Runs when the init button is pressed //
        //////////////////////////////////////////

        if (isStopRequested()) return;
        waitForStart();

        cvTest.start();

        //////////////////////////////////////////
        // Runs when the play button is pressed //
        //////////////////////////////////////////
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        java.util.List<Double> average = new java.util.ArrayList<Double>();
        // Main loop
        while (opModeIsActive()) {
            Vec2 position = cvTest.getSamplePosition();
            if (position == null) {
                continue;
            }

            telemetry.addData("ax", position.x);
            telemetry.addData("ay", position.y);
            telemetry.addData("x", cvTest.analyse().getTx());
            telemetry.addData("y", cvTest.analyse().getTy());
            telemetry.update();
        }
    }
}
