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
}

/*
testTouchSensor = hardwareMap.get(TouchSensor.class, "testTouch");
testMagSensor = hardwareMap.get(TouchSensor.class, "testMag");
*/