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

@TeleOp
public class ServoTuning extends LinearOpMode
{
    @Config
    public static class Tune {
        public static double intakeSpinPower = 0.0;
        public static double intakeLiftPosition = 0.88;
        public static double outtakeArmPosition = 0.03;
        public static double outtakeClawPosition = 0.0;
        public static double latchPosition = 0.0;

        public static double aP = 0.5;
        public static double bI = 0.0;
        public static double cD = 0.0002;
        public static double dTarget = 0.0;

        public static double error = 3.0;

        // arm down 0
        // arm up 0.6
        // arm transfer 0.16
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        PIDController controller = new PIDController(Tune.aP, Tune.bI, Tune.cD);
        Servos servos = new Servos(hardwareMap);
        Sensors sensors = new Sensors(hardwareMap);
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
            double error = Math.abs(motors.intakePosition() - Tune.dTarget);
            double intakeResponse = 0.0;
            if (error > Tune.error) { intakeResponse = controller.calculate(motors.intakePosition(), Tune.dTarget); }
            motors.leftIntakeSlide.set(intakeResponse);
            motors.rightIntakeSlide.set(intakeResponse);
            servos.intakeServoA.set(Tune.intakeSpinPower);
            servos.intakeServoB.set(Tune.intakeSpinPower);
            servos.rightIntakeLiftServo.setPosition(Tune.intakeLiftPosition);
            servos.leftIntakeLiftServo.setPosition(1.0 - Tune.intakeLiftPosition);
            servos.armServoA.setPosition(Tune.outtakeArmPosition);
            servos.armServoB.setPosition(1.0 - Tune.outtakeArmPosition);
            servos.bucketServo.setPosition(Tune.outtakeClawPosition);
            servos.latchServo.setPosition(Tune.latchPosition);
            telemetry.addData("pos", motors.intakePosition());
            telemetry.addData("tar", Tune.dTarget);
            telemetry.update();
        }
    }
}