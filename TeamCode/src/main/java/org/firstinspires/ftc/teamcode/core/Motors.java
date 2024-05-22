package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.core.params.RobotParameters;

public class Motors {
    public Motor leftFront;
    public Motor leftBack;
    public Motor rightFront;
    public Motor rightBack;

    public Motors(HardwareMap hardwareMap) {
        // FTC Lib motor wrappers
        leftFront = new Motor(hardwareMap, RobotParameters.Motors.HardwareMapNames.leftFront);
        leftBack = new Motor(hardwareMap, RobotParameters.Motors.HardwareMapNames.leftBack);
        rightFront = new Motor(hardwareMap, RobotParameters.Motors.HardwareMapNames.rightFront);
        rightBack = new Motor(hardwareMap, RobotParameters.Motors.HardwareMapNames.rightBack);
        // Set motor directions
        leftFront.setInverted(RobotParameters.Motors.Reversed.leftFront);
        leftBack.setInverted(RobotParameters.Motors.Reversed.leftBack);
        rightFront.setInverted(RobotParameters.Motors.Reversed.rightFront);
        rightBack.setInverted(RobotParameters.Motors.Reversed.rightBack);
        // Set braking behaviour
        leftFront.setZeroPowerBehavior(RobotParameters.Motors.zeroPowerBehaviour);
        leftBack.setZeroPowerBehavior(RobotParameters.Motors.zeroPowerBehaviour);
        rightFront.setZeroPowerBehavior(RobotParameters.Motors.zeroPowerBehaviour);
        rightBack.setZeroPowerBehavior(RobotParameters.Motors.zeroPowerBehaviour);
    }
}
