package org.firstinspires.ftc.teamcode.core.state;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ProximitySensor {
    private RevColorSensorV3 sensor;

    public ProximitySensor(HardwareMap hardwareMap) {
        sensor = hardwareMap.get(RevColorSensorV3.class, "outtakeColour");
        sensor.initialize();
        sensor.enableLed(false);
    }

    public void toggleLight(boolean state) {
        sensor.enableLed(state);
    }

    public double getDistance() {
        return sensor.getDistance(DistanceUnit.MM);
    }

    public boolean getLED() {
        return sensor.isLightOn();
    }
}
