package org.firstinspires.ftc.teamcode.core.state;

import org.firstinspires.ftc.teamcode.core.Motors;

public class Orientation {
    public Motors motors;
    double yaw;
    double multiplier;

    public Orientation(Motors m) {
        motors = m;
        motors.leftFront.resetEncoder();
        motors.rightBack.resetEncoder();
        motors.leftBack.resetEncoder();
        motors.rightFront.resetEncoder();
        yaw = 0.0;
        multiplier = 0.00925;
    }

    public void update() {
        yaw = motors.leftOdometry() + motors.rightOdometry();
    }

    public double getYaw() {
        return yaw * multiplier;
    }
    public void zeroYaw() {
        yaw = 0.0;
        motors.leftFront.resetEncoder();
        motors.rightBack.resetEncoder();
        motors.leftBack.resetEncoder();
        motors.rightFront.resetEncoder();
    }
}
