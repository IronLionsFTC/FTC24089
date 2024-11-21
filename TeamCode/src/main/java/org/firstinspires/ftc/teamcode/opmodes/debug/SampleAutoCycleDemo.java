package org.firstinspires.ftc.teamcode.opmodes.debug;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;

@TeleOp
public class SampleAutoCycleDemo extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        AutonomousRobot robot = new AutonomousRobot(telemetry, hardwareMap);
        if (isStopRequested()) return;
        waitForStart();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while (opModeIsActive()) {
            robot.extendIntakeForSample();
            while (!robot.isIntakeExtended()) { robot.update(); }
            robot.closeIntakeClaw();
            while (!robot.isIntakeDone()) { robot.update(); }
            while (!robot.waitForTransfer()) { robot.update(); }
            while (!robot.raiseSlidesForSampleDump()) { robot.update(); }
            while (!robot.areSlidesReadyForSampleDump()) { robot.update(); }
            robot.performDump();
            while (!robot.isDumpDone()) { robot.update(); }
            terminateOpModeNow();
        }
    }
}
