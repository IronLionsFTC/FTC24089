package org.firstinspires.ftc.teamcode.opmodes.debug;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;

@TeleOp
public class SpecimenAutoCycleDemo extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        AutonomousRobot robot = new AutonomousRobot(telemetry, hardwareMap);
        if (isStopRequested()) return;
        waitForStart();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while (opModeIsActive()) {
            robot.extendIntakeForSpecimen();
            while (!robot.isIntakeExtended()) { robot.update(); }
            robot.closeIntakeClaw();
            while (!robot.isIntakeDone()) { robot.update(); }
            while (!robot.waitForTransfer()) { robot.update(); }
            while (!robot.specimenStageOne()) { robot.update(); }
            while (!robot.outtakeStageReady()) { robot.update(); }
            robot.specimenStageTwo();
            while (!robot.outtakeStageReady()) { robot.update(); }
            robot.specimenClip();
            while (!robot.waitForClip()) { robot.update(); }
            robot.specimenStageThree();
            while (!robot.outtakeStageReady()) { robot.update(); }
            robot.endSpecimenCycle();
            while (!robot.specimenCycleDone()) { robot.update(); }
            terminateOpModeNow();
        }
    }
}
