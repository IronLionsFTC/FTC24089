package org.firstinspires.ftc.teamcode.core;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.params.RobotParameters;
import org.firstinspires.ftc.teamcode.core.state.Team;

public class Sensors {
    public RevColorSensorV3 intakeColorSensor;

    public double bCount = 0.0;
    public double rCount = 0.0;
    public double count = 0.0;

    public Sensors(HardwareMap hardwareMap) {
        intakeColorSensor = hardwareMap.get(RevColorSensorV3.class, "intakecolor");
        intakeColorSensor.enableLed(true);
    }

    public double r() { return intakeColorSensor.red() / intakeColorSensor.getRawLightDetected(); }
    public double g() { return intakeColorSensor.green() / intakeColorSensor.getRawLightDetected(); }
    public double b() { return intakeColorSensor.blue() / intakeColorSensor.getRawLightDetected(); }
    public double d() { return intakeColorSensor.getDistance(DistanceUnit.MM); }
}