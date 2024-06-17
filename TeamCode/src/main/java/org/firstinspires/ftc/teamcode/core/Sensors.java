package org.firstinspires.ftc.teamcode.core;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Sensors {
    public RevColorSensorV3 testColourSensor;
    public TouchSensor testTouchSensor;
    public TouchSensor testMagSensor;

    public Sensors(HardwareMap hardwareMap) {
        testColourSensor = hardwareMap.get(RevColorSensorV3.class, "testColour");
        testTouchSensor = hardwareMap.get(TouchSensor.class, "testTouch");
        testMagSensor = hardwareMap.get(TouchSensor.class, "testMag");
    }
}
