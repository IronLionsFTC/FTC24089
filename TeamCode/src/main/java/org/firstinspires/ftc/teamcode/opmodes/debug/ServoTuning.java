package org.firstinspires.ftc.teamcode.opmodes.debug;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Motors;
import org.firstinspires.ftc.teamcode.core.Sensors;
import org.firstinspires.ftc.teamcode.core.Servos;
import org.firstinspires.ftc.teamcode.core.params.RobotParameters;

@TeleOp
public class ServoTuning extends LinearOpMode
{
    @Config
    public static class Tune {
        public static double intakeLiftPosition = RobotParameters.ServoBounds.intakeFolded;
        public static double intakeYawPosition = RobotParameters.ServoBounds.intakeYawZero;
        public static double intakeClawPosition = 0.0;

        public static double aP = 0.0;
        public static double bI = 0.0;
        public static double cD = 0.0;
        public static double dTarget = 0.0;
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        PIDController controller = new PIDController(Tune.aP, Tune.bI, Tune.cD);
        Servos servos = new Servos(hardwareMap);
        Motors motors = new Motors(hardwareMap);

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
        while (opModeIsActive()) {
            controller.setPID(Tune.aP, Tune.bI, Tune.cD);
            double intakeResponse = controller.calculate(motors.intakePosition(), Tune.dTarget);

            motors.leftIntakeSlide.set(intakeResponse);
            motors.rightIntakeSlide.set(intakeResponse);

            servos.intakeLiftServo.setPosition(Tune.intakeLiftPosition);
            servos.intakeYawServo.setPosition(Tune.intakeYawPosition);
            servos.intakeClawServo.setPosition(Tune.intakeClawPosition);

            telemetry.addData("pos", motors.intakePosition());
            telemetry.addData("tar", Tune.dTarget);
            telemetry.update();
        }
    }
}