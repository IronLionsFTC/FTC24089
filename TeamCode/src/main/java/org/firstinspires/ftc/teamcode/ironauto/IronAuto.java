package org.firstinspires.ftc.teamcode.ironauto;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.core.params.RobotParameters;

public class IronAuto {
    public Robot robot;
    public double ticks_to_cm = 0.00508;
    public double forward_power = 0.5;
    public double forward_tolerance = 500.0;

    private MotorEx encoderLeft;
    private MotorEx encoderRight;
    private MotorEx encoderSideways;
    int leftRev = RobotParameters.Odometry.Reversed.left ? -1 : 1;
    int rightRev = RobotParameters.Odometry.Reversed.right ? -1 : 1;
    int sidewaysRev = RobotParameters.Odometry.Reversed.sideways ? -1 : 1;
    private double left_ticks() { return this.encoderLeft.getCurrentPosition() * leftRev; }
    private double right_ticks() { return this.encoderRight.getCurrentPosition() * rightRev; }
    private double sideways_ticks() { return this.encoderSideways.getCurrentPosition() * sidewaysRev; }

    public IronAuto(Robot robot) {
        this.robot = robot;
        this.encoderLeft = new MotorEx(robot.hardwareMap, RobotParameters.Odometry.HardwareMapNames.left);
        this.encoderRight = new MotorEx(robot.hardwareMap, RobotParameters.Odometry.HardwareMapNames.right);
        this.encoderSideways = new MotorEx(robot.hardwareMap, RobotParameters.Odometry.HardwareMapNames.sideways);
    }

    public void move(double rightPower, double forwardPower) {
        robot.drivetrain.componentDrive(rightPower, forwardPower);
    }

    public void move_forward(double cm) {
        double initial_left_ticks = left_ticks();
        double initial_right_ticks = right_ticks();
        double initial_sideways_ticks = sideways_ticks();

        double desired_left_ticks = initial_left_ticks + cm / ticks_to_cm;
        double desired_right_ticks = initial_right_ticks + cm / ticks_to_cm;
        double desired_sideways_ticks = initial_sideways_ticks;

        while (!within(left_ticks(), desired_left_ticks, forward_tolerance)
            && !within(right_ticks(), desired_right_ticks, forward_tolerance))
        {
            move(0.0, forward_power);
        }
        move(0.0,0.0);
    }

    private boolean within(double value, double target, double tolerance) {
        return (value <= target + tolerance && value >= target - tolerance);
    }
}
