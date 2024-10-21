package org.firstinspires.ftc.teamcode.core;
import com.arcrobotics.ftclib.kinematics.Odometry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.core.params.RobotParameters;
import org.firstinspires.ftc.teamcode.opmodes.TuneIntakePID;

public class Sensors {
    public RevColorSensorV3 intakeColorSensor;
    AnalogInput armcoder;

    public Sensors(HardwareMap hardwareMap) {
        intakeColorSensor = hardwareMap.get(RevColorSensorV3.class, "intakecolor");
        intakeColorSensor.enableLed(true);
        armcoder = hardwareMap.get(AnalogInput.class, "armEncoder");
    }

    public double r() { return intakeColorSensor.red() / intakeColorSensor.getRawLightDetected(); }
    public double g() { return intakeColorSensor.green() / intakeColorSensor.getRawLightDetected(); }
    public double b() { return intakeColorSensor.blue() / intakeColorSensor.getRawLightDetected(); }

    public double getArmPosition() {
        double voltage = armcoder.getVoltage();
        return (252.0 - (voltage / 3.33) * 360.0) / 235.0 * 360.0;
    }
}