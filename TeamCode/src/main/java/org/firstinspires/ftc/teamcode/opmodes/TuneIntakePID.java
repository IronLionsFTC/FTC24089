package org.firstinspires.ftc.teamcode.opmodes;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.Sensors;
import org.firstinspires.ftc.teamcode.core.params.RobotParameters;

@TeleOp
public class TuneIntakePID extends LinearOpMode
{
    @Config
    public static class ArmTargetPosition {
        public static double intakeT = 0.5; // 0.49
        public static double armT = 0.0;    // 0.64 top, 0.0 bottom, 0.09 transfer
        public static double bucketPos = 0.0;
        public static double intakePower = 0.0;
        public static double framesInThresh = 4.0;
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        Servo arm = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.armServo);
        Servo intakeA = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.leftIntakeLiftServo);
        Servo intakeB = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.rightIntakeLiftServo);
        CRServo intakeServoA = new CRServo(hardwareMap, RobotParameters.Motors.HardwareMapNames.intakeServoA);
        CRServo intakeServoB = new CRServo(hardwareMap, RobotParameters.Motors.HardwareMapNames.intakeServoB);
        Servo bucket = hardwareMap.get(Servo.class, RobotParameters.Motors.HardwareMapNames.bucketServo);
        Sensors sensors = new Sensors(hardwareMap);
        //////////////////////////////////////////
        // Runs when the init button is pressed //
        //////////////////////////////////////////

        double framesPresent = 0.0;
        double rCount = 0.0;
        double bCount = 0.0;
        double yCount = 0.0;

        if (isStopRequested()) return;
        waitForStart();

        //////////////////////////////////////////
        // Runs when the play button is pressed //
        //////////////////////////////////////////
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // Main loop
        while (opModeIsActive()){

            double dist = sensors.intakeColorSensor.getDistance(DistanceUnit.MM);
            double present = 0.0;
            if (dist < 20) {
                framesPresent += 1.0;
            } else {
                framesPresent = 0.0;
            }

            if (framesPresent >= ArmTargetPosition.framesInThresh) {
                present = 100.0;
            }

            double r = sensors.r();
            double g = sensors.g();
            double b = sensors.b();

            if (b > (r + g) * 1.2) {
                bCount += 1.0;
            } else { bCount = 0.0; }
            if (r > (b + g) * 1.2) {
                rCount += 1.0;
            } else { rCount = 0.0; }

            if (bCount >= ArmTargetPosition.framesInThresh && framesPresent >= ArmTargetPosition.framesInThresh) {
                ArmTargetPosition.intakePower = 0.0;
            }

            arm.setPosition(ArmTargetPosition.armT);
            intakeServoA.set(ArmTargetPosition.intakePower);
            intakeServoB.set(ArmTargetPosition.intakePower);
            intakeA.setPosition(1.0 - ArmTargetPosition.intakeT);
            intakeB.setPosition(ArmTargetPosition.intakeT);
            bucket.setPosition(ArmTargetPosition.bucketPos);
            telemetry.addData("a: BLUE", sensors.b());
            telemetry.addData("b: RED", sensors.r());
            telemetry.addData("c: GREEN", sensors.g());
            telemetry.addData("d: DISTANCE", dist);
            telemetry.addData("e: PRESENT", present);
            telemetry.update();
        }
    }
}