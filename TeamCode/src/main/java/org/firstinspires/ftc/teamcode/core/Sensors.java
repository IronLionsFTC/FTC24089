package org.firstinspires.ftc.teamcode.core;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Sensors {
    public RevColorSensorV3 intakeColorSensor;

    public Sensors(HardwareMap hardwareMap) {
        intakeColorSensor = hardwareMap.get(RevColorSensorV3.class, "intakecolor");
    }

    public double r() { return intakeColorSensor.red() / intakeColorSensor.getRawLightDetected(); }
    public double g() { return intakeColorSensor.green() / intakeColorSensor.getRawLightDetected(); }
    public double b() { return intakeColorSensor.blue() / intakeColorSensor.getRawLightDetected(); }
}