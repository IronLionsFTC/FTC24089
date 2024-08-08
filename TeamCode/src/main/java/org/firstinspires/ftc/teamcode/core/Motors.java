package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.core.params.RobotParameters;

public class Motors {
    // Drive motors
    public Motor leftFront;
    public Motor leftBack;
    public Motor rightFront;
    public Motor rightBack;

    // Intake motors
    public Motor leftIntake;
    public Motor rightIntake;

    // Outake motors
    public Motor leftSlide;
    public Motor rightSlide;

    public Motors(HardwareMap hardwareMap) {
        // FTC Lib motor wrappers1
        leftFront = new Motor(hardwareMap, RobotParameters.Motors.HardwareMapNames.leftFront);
        leftBack = new Motor(hardwareMap, RobotParameters.Motors.HardwareMapNames.leftBack);
        rightFront = new Motor(hardwareMap, RobotParameters.Motors.HardwareMapNames.rightFront);
        rightBack = new Motor(hardwareMap, RobotParameters.Motors.HardwareMapNames.rightBack);
        leftIntake = new Motor(hardwareMap, RobotParameters.Motors.HardwareMapNames.leftIntake);
        rightIntake = new Motor(hardwareMap, RobotParameters.Motors.HardwareMapNames.rightIntake);
        leftSlide = new Motor(hardwareMap, RobotParameters.Motors.HardwareMapNames.leftSlide);
        rightSlide = new Motor(hardwareMap, RobotParameters.Motors.HardwareMapNames.rightSlide);
        // Set motor directions
        leftFront.setInverted(RobotParameters.Motors.Reversed.leftFront);
        leftBack.setInverted(RobotParameters.Motors.Reversed.leftBack);
        rightFront.setInverted(RobotParameters.Motors.Reversed.rightFront);
        rightBack.setInverted(RobotParameters.Motors.Reversed.rightBack);
        leftIntake.setInverted(RobotParameters.Motors.Reversed.leftIntake);
        rightIntake.setInverted(RobotParameters.Motors.Reversed.rightIntake);
        leftSlide.setInverted(RobotParameters.Motors.Reversed.leftSlide);
        rightSlide.setInverted(RobotParameters.Motors.Reversed.rightSlide);
        // Set braking behaviour
        leftFront.setZeroPowerBehavior(RobotParameters.Motors.zeroPowerBehaviour);
        leftBack.setZeroPowerBehavior(RobotParameters.Motors.zeroPowerBehaviour);
        rightFront.setZeroPowerBehavior(RobotParameters.Motors.zeroPowerBehaviour);
        rightBack.setZeroPowerBehavior(RobotParameters.Motors.zeroPowerBehaviour);
        leftIntake.setZeroPowerBehavior(RobotParameters.Motors.zeroPowerBehaviour);
        rightIntake.setZeroPowerBehavior(RobotParameters.Motors.zeroPowerBehaviour);
        leftSlide.setZeroPowerBehavior(RobotParameters.Motors.zeroPowerBehaviour);
        rightSlide.setZeroPowerBehavior(RobotParameters.Motors.zeroPowerBehaviour);
    }
}
