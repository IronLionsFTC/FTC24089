package org.firstinspires.ftc.teamcode.core;
import com.arcrobotics.ftclib.kinematics.Odometry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.params.RobotParameters;
import org.firstinspires.ftc.teamcode.core.state.Team;
import org.firstinspires.ftc.teamcode.opmodes.TuneIntakePID;

public class Sensors {
    public RevColorSensorV3 intakeColorSensor;

    public double bCount = 0.0;
    public double rCount = 0.0;
    public double count = 0.0;

    AnalogInput armcoder;

    public Sensors(HardwareMap hardwareMap) {
        intakeColorSensor = hardwareMap.get(RevColorSensorV3.class, "intakecolor");
        intakeColorSensor.enableLed(true);
        armcoder = hardwareMap.get(AnalogInput.class, "armEncoder");
    }

    public double r() { return intakeColorSensor.red() / intakeColorSensor.getRawLightDetected(); }
    public double g() { return intakeColorSensor.green() / intakeColorSensor.getRawLightDetected(); }
    public double b() { return intakeColorSensor.blue() / intakeColorSensor.getRawLightDetected(); }

    public boolean isMatch(Team team) {
        double r = r();
        double b = b();
        double g = g();
        if (b > (r + g) * 1.2) {
            bCount += 1.0;
        } else { bCount = 0.0; }
        if (r > (b + g) * 1.2) {
            rCount += 1.0;
        } else { rCount = 0.0; }
        if (intakeColorSensor.getDistance(DistanceUnit.MM) < 20.0) {
            count += 1.0;
        } else { count = 0.0; }
        if (team == Team.Blue) {
            return (bCount >= RobotParameters.Thresholds.colourThresh && count >= RobotParameters.Thresholds.colourThresh);
        } else {
            return (rCount >= RobotParameters.Thresholds.colourThresh && count >= RobotParameters.Thresholds.colourThresh);
        }
    }

    public double getArmPosition() {
        double voltage = armcoder.getVoltage();
        return (252.0 - (voltage / 3.33) * 360.0) / 235.0 * 360.0;
    }
}