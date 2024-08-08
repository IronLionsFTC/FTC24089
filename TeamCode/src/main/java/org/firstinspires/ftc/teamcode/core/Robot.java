package org.firstinspires.ftc.teamcode.core;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
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
import org.firstinspires.ftc.teamcode.core.Vec2;
import org.firstinspires.ftc.teamcode.core.Sensors;
import org.firstinspires.ftc.teamcode.core.state.Servos;

public class Robot {
    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    public RobotState state;
    public Controller controller;
    public static Blinkin blinkin;

    public Drivetrain drivetrain;
    public RobotIMU imu;
    public Sensors sensors;
    public States states;

    public Robot(HardwareMap h, Telemetry t) {
        hardwareMap = h;
        telemetry = t;

        state = new RobotState();
        controller = new Controller();
        blinkin = new Blinkin(hardwareMap);

        drivetrain = new Drivetrain(hardwareMap);
        imu = new RobotIMU(hardwareMap);
        sensors = new Sensors(hardwareMap);
        states = new States();
    }

    public class States{
        public class InTake {
            public boolean isRaised = false;
        }
        public class OutTake {
            public boolean isRaised = false;
        }
    }

    public class Drivetrain {
        public Motors motors;
        public Servos servos;
        public MotorPowers MotorPowers;
        public ServoPositions ServoPositions;
        public PairPositions PairPositions;

        public Drivetrain(HardwareMap hardwareMap) {
            motors = new Motors(hardwareMap);
            servos = new Servos(hardwareMap);
            imu = new RobotIMU(hardwareMap);
            MotorPowers = new MotorPowers();
            ServoPositions = new ServoPositions();
            PairPositions = new PairPositions();
            motors.rightSlide.resetEncoder();
            motors.leftSlide.resetEncoder();
        }

        public class MotorPowers {
            public double leftFront = 0.0;
            public double leftBack = 0.0;
            public double rightFront = 0.0;
            public double rightBack = 0.0;
            public double leftIntake = 0.0;
            public double rightIntake = 0.0;
            public double leftSlide = 0.0;
            public double rightSlide = 0.0;
        }

        public class PairPositions {
            public double outTake = 0.0;
            public double outTakeLastError = 0.0;
        }

        public class ServoPositions {
            public double rightIntakeServo = 0.0;
            public double leftIntakeServo =  0.0;
        }

        public void setIntakeServos(double degreesUp) {
            ServoPositions.rightIntakeServo = -degreesUp;
            ServoPositions.leftIntakeServo  =  degreesUp;
        }

        private void componentDrive(double forwardPower, double rightPower) {
            Vec2 powerVec2 = new Vec2();
            double r = imu.yawCorrection();
            double movementMultiplier = 1.0;
//            double denominator = Math.abs(forwardPower) + Math.abs(rightPower);
            powerVec2.fromComponent(rightPower, forwardPower);
            MotorPowers.leftFront = ((rightPower * -1 + forwardPower) * movementMultiplier + r);
            MotorPowers.rightFront = ((rightPower + forwardPower) * movementMultiplier - r);
            MotorPowers.leftBack = ((rightPower + forwardPower) * movementMultiplier + r);
            MotorPowers.rightBack = ((rightPower * -1 + forwardPower) * movementMultiplier - r);
        }

        private void driveInDirection(double direction, double power, boolean fieldCentric) {
            double degrees = direction - 90;
            if (fieldCentric) { degrees -= imu.getYawDegrees(); }
            if (degrees > 180.0) { degrees -= 360.0; }
            if (degrees <-180.0) { degrees += 360.0; }
            Vec2 driveVector = new Vec2();
            driveVector.fromPolar(power, degrees);
            componentDrive(driveVector.y, driveVector.x);
        }

        public void stopMotors() {
            MotorPowers.leftFront = 0.0;
            MotorPowers.leftBack = 0.0;
            MotorPowers.rightBack = 0.0;
            MotorPowers.rightFront = 0.0;
            setMotorPowers();
        }

        public void movePairs() {
            // 0.0 <= TargetDegrees <= 1.0
            double targetDegrees = PairPositions.outTake * RobotParameters.maxOutTakeEncoder;
            double currentDegrees = (motors.leftSlide.getCurrentPosition() + motors.rightSlide.getCurrentPosition()) * 0.5;
            double error = targetDegrees - currentDegrees;
            // If for whatever reason the slides can't go all the way down
            // then ensure they aren't pulling fruitlessly.
            if (error < -10 || error > 0) {
                double response = imu.calculate_PID(0.75, 0.35, error, PairPositions.outTakeLastError);
                // If not moving down, add the feed-forward to hold weight.
                if (targetDegrees < 10.0 && response > -0.1) {
                    response += RobotParameters.slideWeightCompensation;
                }
                MotorPowers.rightSlide = response;
                MotorPowers.leftSlide = response;
            }
        }

        // TODO: check later
        public void calculateMovement(GamepadEx gamepad) {
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

            if (Controls.driverOverride.isPressed(gamepad)) {
                return;
            }

            double intakePower = 0.0;
            if (gamepad.getButton(GamepadKeys.Button.X)) {
                intakePower = 1.0;
            }
            setIntakeServos(controller.left_trigger(gamepad) * 30);
            MotorPowers.leftIntake = intakePower;
            MotorPowers.rightIntake = intakePower;
            componentDrive(my * 0.7, mx * 0.7);
        }

        public void setMotorPowers() {
            motors.leftFront.set(MotorPowers.leftFront);
            motors.rightFront.set(MotorPowers.rightFront);
            motors.leftBack.set(MotorPowers.leftBack);
            motors.rightBack.set(MotorPowers.rightBack);
            motors.leftIntake.set(MotorPowers.leftIntake);
            motors.rightIntake.set(MotorPowers.rightIntake);
            motors.leftSlide.set(MotorPowers.leftSlide);
            motors.rightSlide.set(MotorPowers.rightSlide);
        }

        public void setServoPositions() {
            servos.rightIntakeServo.set(ServoPositions.rightIntakeServo);
            servos.leftIntakeServo.set(ServoPositions.leftIntakeServo);
        }

        public void drive(GamepadEx gamepad) {
            calculateMovement(gamepad);
            setServoPositions();
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
            return ((currentError * kP) + (kD * (currentError - lastError))) * -1;
        }

        public double yawCorrection() {
            double rawError = targetYaw - getYawDegrees();
            if (rawError <= -180.0) { rawError += 360.0; }
            if (rawError > 180.0) { rawError -= 360.0; }
            if (rawError >= RobotParameters.IMU.PIDcorrectionThreshold) {
                blinkin.setRightLights(RevBlinkinLedDriver.BlinkinPattern.RED);
                blinkin.setLeftLights(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }
            else if (rawError <= -RobotParameters.IMU.PIDcorrectionThreshold) {
                blinkin.setRightLights(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                blinkin.setLeftLights(RevBlinkinLedDriver.BlinkinPattern.RED);
            }
            else {
                blinkin.setRightLights(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                blinkin.setLeftLights(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }
            if (Math.abs(rawError) > RobotParameters.IMU.PIDcorrectionThreshold) {
                // Now calculate PID.
                double response = calculate_PID(0.75, 0.35, rawError, lastError) / 30;
                lastError = rawError;
                if (response > 1.0) { response = 1.0; }
                if (response <-1.0) { response =-1.0; }
                if (response > -0.1 && response < 0.0) {
                    response = -0.1;
                }
                if (response < 0.1 && response > 0.0) {
                    response = 0.1;
                }
                return response * RobotParameters.IMU.correctionMultiplier;
            }
            else {
                return 0.0;
            }
        }
    }
}
