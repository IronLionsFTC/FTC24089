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
        drivetrain.PositionTracker.position.fromComponent(0.0, 0.0);
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
        public PositionTracker PositionTracker;

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

        public class PositionTracker {
            public Vec2 position = new Vec2();
            public double fr = 0.0;
            public double fl = 0.0;
            public double br = 0.0;
            public double bl = 0.0;
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
            if (currentDegrees > 20 || error > 0) {
                double response = imu.calculate_PID(0.65, 0.25, error, PairPositions.outTakeLastError);
                // If not moving down, add the feed-forward to hold weight.
                if (targetDegrees > 10.0 && response > -0.1) {
                    response += RobotParameters.slideWeightCompensation;
                }
                MotorPowers.rightSlide = response * 0.5;
                MotorPowers.leftSlide = response * 0.5;
            } else {
                MotorPowers.rightSlide = 0.0;
                MotorPowers.leftSlide = 0.0;
            }
        }

        public void calculateMovementVectorFromWheelRotations() {
            double w1Rad = Math.toRadians(motors.leftFront.getCurrentPosition() - PositionTracker.fl);
            PositionTracker.fl = motors.leftFront.getCurrentPosition();
            double w2Rad = Math.toRadians(motors.rightFront.getCurrentPosition() - PositionTracker.fr);
            PositionTracker.fr = motors.rightFront.getCurrentPosition();
            double w3Rad = Math.toRadians(motors.leftBack.getCurrentPosition() - PositionTracker.bl);
            PositionTracker.bl = motors.leftBack.getCurrentPosition();
            double w4Rad = Math.toRadians(motors.rightBack.getCurrentPosition() - PositionTracker.br);
            PositionTracker.br = motors.rightBack.getCurrentPosition();
            double distanceW1 = RobotParameters.radius * w1Rad;
            double distanceW2 = RobotParameters.radius * w2Rad;
            double distanceW3 = RobotParameters.radius * w3Rad;
            double distanceW4 = RobotParameters.radius * w4Rad;
            double dx = (distanceW1 - distanceW2 - distanceW3 + distanceW4) / 4.0;
            double dy = (distanceW1 + distanceW2 - distanceW3 - distanceW4) / 4.0;
            Vec2 displacement = new Vec2();
            displacement.fromComponent(dx, dy);
            // At this point add handling for if the robot is not in the same direction as when it started but im too lazy rn
            PositionTracker.position = PositionTracker.position.add(displacement);
        }

        // TODO: check later
        public void calculateMovement(GamepadEx gamepad) {
            double mx = controller.movement_x(gamepad);
            double my = controller.movement_y(gamepad);
            double controllerR = controller.yawRotation(gamepad);
            double controllerR2 = controller.pitchRotation(gamepad);

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
            if (controller.xPress == 1.0) {
                intakePower = 1.0 - MotorPowers.leftIntake;
            }
            if (controller.yPress == 1.0) {
                PairPositions.outTake = 200.0 - PairPositions.outTake;
            }
            controller.updateKeyTracker(gamepad);
            if (controller.aPress == 1.0) {
                setIntakeServos(20.0 - ServoPositions.leftIntakeServo);
            }
            MotorPowers.leftIntake = intakePower;
            MotorPowers.rightIntake = intakePower;
            componentDrive(my, mx);

            // Experimental position tracking
            calculateMovementVectorFromWheelRotations();
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
            movePairs();
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
                return response * RobotParameters.IMU.correctionMultiplier * -1.0;
            }
            else {
                return 0.0;
            }
        }
    }
}
