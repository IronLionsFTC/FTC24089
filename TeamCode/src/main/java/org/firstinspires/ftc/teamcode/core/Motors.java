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
    public Motor intakeSlide;

    // Outake motors
    public Motor leftOuttakeSlide;
    public Motor rightOuttakeSlide;

    // Motor Powers
    public MotorPowers powers;

    public Motors(HardwareMap hardwareMap) {
        leftFront = new Motor(hardwareMap, RobotParameters.Motors.HardwareMapNames.leftFront);
        leftBack = new Motor(hardwareMap, RobotParameters.Motors.HardwareMapNames.leftBack);
        rightFront = new Motor(hardwareMap, RobotParameters.Motors.HardwareMapNames.rightFront);
        rightBack = new Motor(hardwareMap, RobotParameters.Motors.HardwareMapNames.rightBack);
        intakeSlide =  new Motor(hardwareMap, RobotParameters.Motors.HardwareMapNames.rightIntakeSlide);
        leftOuttakeSlide =  new Motor(hardwareMap, RobotParameters.Motors.HardwareMapNames.leftOuttakeSlide);
        rightOuttakeSlide =  new Motor(hardwareMap, RobotParameters.Motors.HardwareMapNames.rightOuttakeSlide);

        // Set motor directions
        leftFront.setInverted(RobotParameters.Motors.Reversed.leftFront);
        leftBack.setInverted(RobotParameters.Motors.Reversed.leftBack);
        rightFront.setInverted(RobotParameters.Motors.Reversed.rightFront);
        rightBack.setInverted(RobotParameters.Motors.Reversed.rightBack);
        leftOuttakeSlide.setInverted(false);
        rightOuttakeSlide.setInverted(true);
        intakeSlide.setInverted(true);

        // Set braking behaviour
        leftFront.setZeroPowerBehavior(RobotParameters.Motors.zeroPowerBehaviour);
        leftBack.setZeroPowerBehavior(RobotParameters.Motors.zeroPowerBehaviour);
        rightFront.setZeroPowerBehavior(RobotParameters.Motors.zeroPowerBehaviour);
        rightBack.setZeroPowerBehavior(RobotParameters.Motors.zeroPowerBehaviour);
        leftOuttakeSlide.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        intakeSlide.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightOuttakeSlide.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);


        rightOuttakeSlide.resetEncoder();
        leftOuttakeSlide.resetEncoder();
        intakeSlide.resetEncoder();

        // Initialize Tracking Classes
        powers = new MotorPowers();

    }

    public void zeroEncoders() {
        rightBack.resetEncoder();
        leftFront.resetEncoder();
    }

    public static class MotorPowers {
        public double leftFront = 0.0;
        public double leftBack = 0.0;
        public double rightFront = 0.0;
        public double rightBack = 0.0;
        public double leftOuttakeSlide = 0.0;
        public double rightOuttakeSlide = 0.0;
        public double leftIntakeSlide = 0.0;
        public double rightIntakeSlide = 0.0;
    }

    public double leftOdometry() {
        return rightBack.getCurrentPosition();
    }

    public double rightOdometry() {
        return leftFront.getCurrentPosition();
    }

    public void setDrivePowers() {
        leftFront.set(Math.abs(powers.leftFront) > 0.1 ? powers.leftFront : 0.0);
        rightFront.set(Math.abs(powers.rightFront) > 0.1 ? powers.rightFront : 0.0);
        leftBack.set(Math.abs(powers.leftBack) > 0.1 ? powers.leftBack : 0.0);
        rightBack.set(Math.abs(powers.rightBack) > 0.1 ? powers.rightBack : 0.0);
    }
    public void setOtherPowers() {
        leftOuttakeSlide.set(Math.abs(powers.leftOuttakeSlide) > 0.1 ? powers.leftOuttakeSlide : 0.0);
        rightOuttakeSlide.set(Math.abs(powers.rightOuttakeSlide) > 0.1 ? powers.rightOuttakeSlide : 0.0);
        intakeSlide.set(Math.abs(powers.leftIntakeSlide) > 0.1 ? powers.leftIntakeSlide : 0.0);
    }

    public void stopMotors() {
        powers.leftBack = 0.0;
        powers.rightBack = 0.0;
        powers.leftFront = 0.0;
        powers.rightFront = 0.0;
        setDrivePowers();
    }

    public double outtakePosition() {
        return (rightOuttakeSlide.getCurrentPosition() + leftOuttakeSlide.getCurrentPosition()) * 0.5;
    }

    public double intakePosition() {
        return intakeSlide.getCurrentPosition();
    }
}
