package org.firstinspires.ftc.teamcode.opmodes.debug;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.core.Sensors;
import org.firstinspires.ftc.teamcode.core.params.RobotParameters;
import org.firstinspires.ftc.teamcode.core.state.Colour;
import org.firstinspires.ftc.teamcode.core.state.ColourProfile;

@TeleOp
public class ColourProfileTuning extends LinearOpMode
{
    @Config
    public static class ColourThreshold {
        public static double intakePower = 0.0;
        public static int targetColour = 0; // 0=Never stop 1=Stop on red 2=Stop on blue 3=Stop on yellow 4=always stop
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        CRServo intakeServoA = new CRServo(hardwareMap, RobotParameters.Motors.HardwareMapNames.intakeServoA);
        CRServo intakeServoB = new CRServo(hardwareMap, RobotParameters.Motors.HardwareMapNames.intakeServoB);
        Sensors sensors = new Sensors(hardwareMap);

        //////////////////////////////////////////
        // Runs when the init button is pressed //
        //////////////////////////////////////////

        ColourProfile colourProfile = new ColourProfile(sensors);

        if (isStopRequested()) return;
        waitForStart();

        //////////////////////////////////////////
        // Runs when the play button is pressed //
        //////////////////////////////////////////
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // Main loop
        while (opModeIsActive()){

            colourProfile.update(sensors);

            telemetry.addData("a: BLUE", colourProfile.b);
            telemetry.addData("b: RED", colourProfile.r);
            telemetry.addData("c: GREEN", colourProfile.g);
            telemetry.addData("d: DISTANCE", colourProfile.d);

            Colour current = colourProfile.classify();

            if (current == Colour.Red && ColourThreshold.targetColour == 1) {
                intakeServoA.set(0.0);
                intakeServoB.set(0.0);
            } else if (current == Colour.Blue && ColourThreshold.targetColour == 2) {
                intakeServoA.set(0.0);
                intakeServoB.set(0.0);
            } else if (current == Colour.Yellow && ColourThreshold.targetColour == 3) {
                intakeServoA.set(0.0);
                intakeServoB.set(0.0);
            } else {
                intakeServoA.set(ColourThreshold.intakePower);
                intakeServoB.set(ColourThreshold.intakePower);
            }

            telemetry.update();
        }
    }
}