package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.core.auxiliary.Blinkin;
import org.firstinspires.ftc.teamcode.core.params.RobotParameters;
import org.firstinspires.ftc.teamcode.core.state.RobotState;

import org.firstinspires.ftc.teamcode.core.params.Controls;

public class Robot {
    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    public RobotState state;
    public Controller controller;
    public Blinkin blinkin;

    public Drivetrain drivetrain;
    public RobotIMU imu;

    public Robot(HardwareMap h, Telemetry t) {
        hardwareMap = h;
        telemetry = t;

        state = new RobotState();
        controller = new Controller();
        blinkin = new Blinkin(hardwareMap);

        drivetrain = new Drivetrain(hardwareMap);
        imu = new RobotIMU(hardwareMap);
    }

    public class Drivetrain {
        public Motors motors;
        public DesiredMovements desiredMovements;

        public Drivetrain(HardwareMap hardwareMap) {
            motors = new Motors(hardwareMap);
            imu = new RobotIMU(hardwareMap);
            desiredMovements = new DesiredMovements();
        }

        public class DesiredMovements {
            public double leftFront = 0.0;
            public double leftBack = 0.0;
            public double rightFront = 0.0;
            public double rightBack = 0.0;
        }

        public void setAllPower(double power) {
            desiredMovements.leftFront = power;
            desiredMovements.leftBack = power;
            desiredMovements.rightFront = power;
            desiredMovements.rightBack = power;
        }

        public void addFlBrDiagonal(double power) {
            desiredMovements.leftFront += power;
            desiredMovements.rightBack += power;
        }
        public void addFrBlDiagonal(double power) {
            desiredMovements.rightFront += power;
            desiredMovements.leftBack += power;
        }

        // TODO: check later
        public void driveInDirection(double degrees, double power) {
            degrees -= imu.getYawDegrees();
            Vector driveVector;
            if (degrees < -180) {
                degrees += 360;
            }
            if (degrees > 180) {
                degrees -= 360;
            }
            driveVector = Vector.from_compass(power, degrees);
            // ^^ gives the vector in which we need to drive
            setAllPower(driveVector.y);

            addFrBlDiagonal(driveVector.x * -1);
            addFlBrDiagonal(driveVector.x);
            setMotorPowers();
        }

        public void stopMotors() {
            setAllPower(0.0);
        }

        // TODO: check later
        public void calculateMovementTele(GamepadEx gamepad) {
            double mx = controller.movement_x(gamepad);
            double my = controller.movement_y(gamepad);
            double controllerR = controller.rotation(gamepad);

            imu.targetYaw -= controllerR * 2.0; // Assuming -1 -> 1
            if (imu.targetYaw < -180) {
                imu.targetYaw += 360;
            }
            if (imu.targetYaw > 180) {
                imu.targetYaw -= 360;
            }

            double r = imu.calculateRotationPower();

            desiredMovements.leftFront = my + mx + r;//((mx + my) * movementMultiplier + r) / denominator;
            desiredMovements.rightFront = my - mx - r;//((mx - my) * movementMultiplier - r) / denominator;
            desiredMovements.leftBack = my - mx + r;//((mx - my) * movementMultiplier + r) / denominator;
            desiredMovements.rightBack = my + mx - r;//((mx + my) * movementMultiplier - r) / denominator;

            // Driver movement override
            if (Controls.driverOverride.isPressed(gamepad)) {
                return;
            }

            Vector controllerVector = Vector.from_components(mx, my);
            telemetry.addData("CONTROLLER VECTOR", controllerVector.direction());
            telemetry.update();
            driveInDirection(controllerVector.direction(), RobotParameters.Movement.speed);
        }

        public void setMotorPowers() {
            motors.leftFront.set(desiredMovements.leftFront);
            motors.rightFront.set(desiredMovements.rightFront);
            motors.leftBack.set(desiredMovements.leftBack);
            motors.rightBack.set(desiredMovements.rightBack);
        }

        public void moveTele(GamepadEx gamepad) {
            calculateMovementTele(gamepad);
            setMotorPowers();
        }
    }

    public static class RobotIMU {
        public IMU imu;
        public double targetYaw = 0.0;
        public double lastError = 0.0;

        public RobotIMU(HardwareMap hardwareMap) {
            imu = hardwareMap.get(
                    com.qualcomm.robotcore.hardware.IMU.class,
                    RobotParameters.IMU.hardwareMapName
            );
            imu.initialize(
                    new IMU.Parameters(
                            RobotParameters.IMU.hubOrientation
                    )
            );
            resetYaw();
        }

        public void resetYaw() {
            imu.resetYaw();
        }

        public double getYawDegrees() {
            Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            return AngleUnit.DEGREES.normalize(angles.firstAngle);
        }

        public double calculate_PID(double kP, double kD, double currentError, double lastError) {
            // Do not change this.
            return ((currentError * kP) + (kD * (currentError - lastError))) * -0.1;
        }

        // TODO: check later
        public double calculateRotationPower(double threshold) {
            double currentRotation = getYawDegrees();
            double error = targetYaw - currentRotation;

            if (Math.abs(error) > 180.0) {
                if (currentRotation > 0 && targetYaw < 0) {
                    error = (180 - currentRotation) + (180 + targetYaw);
                } else if (currentRotation < 0 && targetYaw > 0) {
                    error = (-180 - currentRotation) + (targetYaw - 180);
                }
            }

            double correction = 0;
            if (Math.abs(error) > threshold) {
                correction = calculate_PID(0.5, 0.5, error, lastError);
            }
            correction *= RobotParameters.IMU.correctionMultiplier;
            if (correction < -1) {
                correction = -1;
            }
            if (correction > 1) {
                correction = 1;
            }

            lastError = error;

            double mt = RobotParameters.IMU.rotationMinimumThreshold;
            return (correction > -mt && correction < mt) ? 0.0 // Handle division by zero
                    : (correction + (Math.abs(correction) / correction) * 0.02) * RobotParameters.IMU.rotationSpeedMultiplier;
        }
        public double calculateRotationPower() {
            return calculateRotationPower(RobotParameters.IMU.PIDcorrectionThreshold);
        }
    }
}
