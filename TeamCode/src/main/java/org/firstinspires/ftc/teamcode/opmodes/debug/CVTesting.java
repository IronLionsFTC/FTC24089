package org.firstinspires.ftc.teamcode.opmodes.debug;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.teamcode.core.Motors;
import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.core.Sensors;
import org.firstinspires.ftc.teamcode.core.Servos;
import org.firstinspires.ftc.teamcode.core.Vec2;
import org.firstinspires.ftc.teamcode.core.params.RobotParameters;
import org.firstinspires.ftc.teamcode.core.state.ComputerVision;
import org.firstinspires.ftc.teamcode.core.state.QuadrilateralTracker;

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

        // Main loop
        while (opModeIsActive()) {
            LLResult result = cvTest.analyse();
            if (result != null) cvTest.sample.update(cvTest.getSampleCornerPositions(result));
            double rotation = cvTest.sample.getDirection();
            telemetry.addData("rot", rotation);
            servos.intakeYawServo.setPosition(RobotParameters.ServoBounds.intakeYawZero + rotation);
            telemetry.update();
        }
    }
}
