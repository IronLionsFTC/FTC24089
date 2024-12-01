package org.firstinspires.ftc.teamcode.core.state;

import org.firstinspires.ftc.teamcode.core.Motors;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class Orientation {
    public Motors motors;
    double yaw;
    double multiplier;
    double lastYaw;
    double velocity;
    Timer timer = new Timer();

    public Orientation(Motors m) {
        motors = m;
        motors.leftFront.resetEncoder();
        motors.rightBack.resetEncoder();
        motors.leftBack.resetEncoder();
        motors.rightFront.resetEncoder();
        yaw = 0.0;
        lastYaw = yaw;
        multiplier = 0.00925;
    }

    public void update() {
        yaw = motors.leftOdometry() + motors.rightOdometry();
        double delta = lastYaw - getYaw();
        velocity = delta / timer.getElapsedTimeSeconds();
        timer.resetTimer();
        lastYaw = getYaw();
    }

    public double getYaw() {
        return yaw * multiplier;
    }
    public double getVelocity() { return velocity; }
    public void zeroYaw() {
        yaw = 0.0;
        motors.leftFront.resetEncoder();
        motors.rightBack.resetEncoder();
        motors.leftBack.resetEncoder();
        motors.rightFront.resetEncoder();
    }
}
