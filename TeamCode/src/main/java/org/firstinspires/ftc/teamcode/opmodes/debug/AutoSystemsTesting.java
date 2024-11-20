package org.firstinspires.ftc.teamcode.opmodes.debug;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.core.control.Controller;
import org.firstinspires.ftc.teamcode.core.Robot;
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
        AutonomousRobot robot = new AutonomousRobot(telemetry, hardwareMap);
        if (isStopRequested()) return;
        waitForStart();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while (opModeIsActive()) {
            robot.extendIntake();
            while (!robot.isIntakeExtended()) { robot.update(); }
            robot.closeIntakeClaw();
            while (!robot.isGrabbingDone()) { robot.update(); }
            robot.retractIntake();
            while (!robot.isTransferReady()) { robot.update(); }
            while (!robot.waitForTransfer()) { robot.update(); }
            robot.outtakeStageOne();
            while (!robot.outtakeStageReady()) { robot.update(); }
            robot.outtakeStageTwo();
            while (!robot.outtakeStageReady()) { robot.update(); }
            robot.specimenClip();
            while (!robot.specimenCycleDone()) { robot.update(); }
            robot.endSpecimenCycle();
            terminateOpModeNow();
        }
    }
}
