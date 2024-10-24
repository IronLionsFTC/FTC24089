package org.firstinspires.ftc.teamcode.opmodes.debug;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.core.Sensors;
import org.firstinspires.ftc.teamcode.core.Servos;

@TeleOp
public class ServoTuning extends LinearOpMode
{
    @Config
    public static class Tune {
        public static double intakeSpinPower = 0.0;
        public static double intakeLiftPosition = 0.0;
        public static double outtakeArmPosition = 0.0;
        public static double outtakeClawPosition = 0.0;
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        Servos servos = new Servos(hardwareMap);
        Sensors sensors = new Sensors(hardwareMap);

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
            servos.intakeServoA.set(Tune.intakeSpinPower);
            servos.intakeServoB.set(Tune.intakeSpinPower);
            servos.rightIntakeLiftServo.setPosition(Tune.intakeLiftPosition);
            servos.leftIntakeLiftServo.setPosition(1.0 - Tune.intakeLiftPosition);
            servos.armServo.setPosition(Tune.outtakeArmPosition);
            servos.bucketServo.setPosition(Tune.outtakeClawPosition);
        }
    }
}