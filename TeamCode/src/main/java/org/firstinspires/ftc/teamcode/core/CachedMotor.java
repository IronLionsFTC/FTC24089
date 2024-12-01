package org.firstinspires.ftc.teamcode.core;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.params.RobotParameters;

public class CachedMotor {
    private Motor hardware;
    private double powerValue = 0.0;
    private double position = 0.0;

    public CachedMotor(HardwareMap hardwareMap, String name) {
        hardware = new Motor(hardwareMap, name);
        hardware.resetEncoder();
    }

    public void setRunMode(Motor.RunMode runmode) {
        hardware.setRunMode(runmode);
    }

    public void setInverted(boolean reversed) {
        hardware.setInverted(reversed);
    }

    public void set(double newPower) {
        if (newPower != powerValue) {
            powerValue = newPower;
            hardware.set(powerValue);
        }
    }

    public void updatePosition() {
        position = hardware.getCurrentPosition();
    }

    public double getCurrentPosition() {
        return position;
    }

    public void setZeroPowerBehavior(Motor.ZeroPowerBehavior mode) {
        hardware.setZeroPowerBehavior(mode);
    }

    public void resetEncoder() {
        hardware.resetEncoder();
    }
}
