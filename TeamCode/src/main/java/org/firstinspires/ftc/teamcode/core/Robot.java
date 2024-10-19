package org.firstinspires.ftc.teamcode.core;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.core.auxiliary.Blinkin;
import org.firstinspires.ftc.teamcode.core.params.RobotParameters;
import org.firstinspires.ftc.teamcode.core.state.RobotState;
import org.firstinspires.ftc.teamcode.core.state.Team;
import org.firstinspires.ftc.teamcode.core.state.intake.IntakeState;
import org.firstinspires.ftc.teamcode.core.state.outtake.OuttakeState;

public class Robot {
    public Team team;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public Controller controller;
    public static Blinkin blinkin;
    public Drivetrain drivetrain;
    public RobotIMU imu;
    public Sensors sensors;
    public RobotState state = new RobotState();
    public PID_settings pidSettings = new PID_settings();

    public Robot(HardwareMap h, Telemetry t, Team colour) {
        hardwareMap = h;
        telemetry = t;

        state = new RobotState();
        controller = new Controller();
        blinkin = new Blinkin(hardwareMap);

        drivetrain = new Drivetrain(hardwareMap);
        // drivetrain.PositionTracker.position.fromComponent(0.0, 0.0);
        imu = new RobotIMU(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        sensors = new Sensors(hardwareMap);
        team = colour;
    }
    @Config
    public static class PID_settings {
        public static double ticks_in_degree = 700.0 / 180.0;
        public static double outtakeSlide_p = 0.002;
        public static double outtakeSlide_i = 0.003;
        public static double outtakeSlide_d = 0.0001;
        public static double outtakeSlide_f = 0.1;

        public static double intakeSlide_p = 0.05;
        public static double intakeSlide_i = 0.05;
        public static double intakeSlide_d = 0.001;
        public static double intakeSlide_f = 0.0;

        public static double yaw_p = 0.03;
        public static double yaw_i = 0.05;
        public static double yaw_d = 0.001;

        public static double clawPos = 0.0;
        public static double armPos = 0.0;
        public static double intakePosition = 0.56;
        public static double intakeSpeed = 0.25;

        public PIDController outtakeSlideController = new PIDController(outtakeSlide_p, outtakeSlide_i, outtakeSlide_d);
        public PIDController intakeSlideController = new PIDController(intakeSlide_d, intakeSlide_i, intakeSlide_d);
        public PIDController yawController = new PIDController(yaw_p, yaw_i, yaw_d);
    }

    public class Drivetrain {
        public Motors motors;
        public Servos servos;
        public PositionTracker PositionTracker;

        public Drivetrain(HardwareMap hardwareMap) {
            motors = new Motors(hardwareMap);
            servos = new Servos(hardwareMap);
            imu = new RobotIMU(hardwareMap);
        }

        public class PositionTracker {
            public Vec2 position = new Vec2();
            public double fr = 0.0;
            public double fl = 0.0;
            public double br = 0.0;
            public double bl = 0.0;
        }

        private void componentDrive(double forwardPower, double rightPower) {
            Vec2 powerVec2 = new Vec2();
            double r = newYawCorrection();
            double movementMultiplier = 0.7;
            if (state.intake.intakeState == IntakeState.Collecting) {
                movementMultiplier = 0.3;
            }
            if (state.outtake.outtakeState == OuttakeState.Up) {
                movementMultiplier = 0.2;
            }
            powerVec2.fromComponent(rightPower, forwardPower);
            motors.powers.leftFront = ((rightPower + forwardPower) * movementMultiplier + r);
            motors.powers.rightFront = ((-rightPower + forwardPower) * movementMultiplier - r);
            motors.powers.leftBack = ((-rightPower - forwardPower) * movementMultiplier + r);
            motors.powers.rightBack = ((rightPower - forwardPower) * movementMultiplier - r);
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

        public void moveOuttake() {
            double outtakeSlidePos = (motors.leftOuttakeSlide.getCurrentPosition() + motors.rightOuttakeSlide.getCurrentPosition()) * 0.5;
            pidSettings.outtakeSlideController.setPID(pidSettings.outtakeSlide_p, pidSettings.outtakeSlide_i, pidSettings.outtakeSlide_d);
            double slideTarget = 0.0;
            if ((state.outtake.outtakeState == OuttakeState.Up || state.outtake.outtakeState == OuttakeState.Deposit) && state.intake.intakeState == IntakeState.Retracted) {
                double intakeSlidePos = (motors.leftIntakeSlide.getCurrentPosition() + motors.rightIntakeSlide.getCurrentPosition()) * 0.5;
                if (intakeSlidePos > 40 || outtakeSlidePos > 1000) {
                    slideTarget = 2950.0;
                }
            } else {
                slideTarget = 0.0;
            }

            double outtakeSlideResponse = pidSettings.outtakeSlideController.calculate(outtakeSlidePos, slideTarget);
            double outtakeSlideFeedForward = Math.cos(Math.toRadians(slideTarget / pidSettings.ticks_in_degree)) * pidSettings.outtakeSlide_f;
            double outtakeSlidePower = outtakeSlideResponse + outtakeSlideFeedForward;
            if (slideTarget <= 10.0 && outtakeSlidePos < 30) { outtakeSlidePower = 0.0; }
            motors.powers.leftOuttakeSlide = outtakeSlidePower;
            motors.powers.rightOuttakeSlide = outtakeSlidePower;
        }

        public void moveIntake() {
            if (sensors.intakeColorSensor.getDistance(DistanceUnit.MM) < 25 && state.intake.intakeState == IntakeState.Collecting) {
                state.intake.intakeState = IntakeState.Evaluating;
            }
            pidSettings.intakeSlideController.setPID(pidSettings.intakeSlide_p, pidSettings.intakeSlide_i, pidSettings.intakeSlide_d);
            double intakeTarget = 0.0;
            double intakeLift = 0.0;
            if (state.intake.intakeState == IntakeState.Folded) {
                intakeTarget = 0.0;
                intakeLift = 0.0;
            } else if (state.intake.intakeState == IntakeState.Retracted || state.intake.intakeState == IntakeState.Folded) {
                if (state.outtake.outtakeState != OuttakeState.Down) {
                    intakeTarget = 60.0;
                } else {
                    if (motors.rightOuttakeSlide.getCurrentPosition() + motors.leftOuttakeSlide.getCurrentPosition() < 200.0) {
                        intakeTarget = 0.0;
                    } else {
                        intakeTarget = 70.0;
                    }
                }
                if (state.outtake.outtakeState == OuttakeState.Up || state.outtake.outtakeState == OuttakeState.Deposit) {
                    if (motors.rightOuttakeSlide.getCurrentPosition() > 1500.0 && motors.leftOuttakeSlide.getCurrentPosition() > 1500.0) {
                        intakeTarget = 0.0;
                    }
                }
                intakeLift = 0.0;
                // servos.intakeLiftServo.setPosition(0.0);
            } else if (state.intake.intakeState == IntakeState.Extended) {
                intakeTarget = 100.0;
                intakeLift = 0.0;
                // servos.intakeLiftServo.setPosition(0.95);
            } else if (state.intake.intakeState == IntakeState.Collecting) {
                intakeTarget = 100.0;
                if (controller.yPress == 0) {
                    intakeLift = pidSettings.intakePosition;
                } else {
                    intakeLift = pidSettings.intakePosition - 0.1;
                }
                // servos.intakeLiftServo.setPosition(0.95);
            } else if (state.intake.intakeState == IntakeState.Evaluating) {
                intakeTarget = 100.0;
                intakeLift = pidSettings.intakePosition;
                // servos.intakeLiftServo.setPosition(0.0);
            } else if (state.intake.intakeState == IntakeState.Depositing || state.intake.intakeState == IntakeState.Dropping) {
                intakeTarget = 28.0;
                intakeLift = 0.0;
                // servos.intakeLiftServo.setPosition(0.0);
            }

            servos.leftIntakeLiftServo.setPosition(1.0 - intakeLift);
            servos.rightIntakeLiftServo.setPosition(intakeLift);
            if (state.intake.intakeState == IntakeState.Evaluating) {
                double r = sensors.r();
                double g = sensors.g();
                double b = sensors.b();
                if (team == Team.Red) {
                    if (r > (g + b) || ((r + g) > b * 2.0 && r < g)) {
                        state.intake.intakeState = IntakeState.Depositing;
                    } else {
                        state.intake.intakeState = IntakeState.Collecting;
                    }
                } else {
                    if (b > (r + g) || ((r + g) > b * 2.0 && r < g)) {
                        state.intake.intakeState = IntakeState.Depositing;
                    } else {
                        state.intake.intakeState = IntakeState.Collecting;
                    }
                }
            }
            if (state.intake.intakeState == IntakeState.Depositing) {
                if (state.outtake.outtakeState == OuttakeState.Up) {
                    state.intake.intakeState = IntakeState.Dropping;
                }
            }
            if (state.intake.intakeState == IntakeState.Dropping) {
                if (sensors.intakeColorSensor.getDistance(DistanceUnit.MM) > 25.0) {
                    state.intake.intakeState = IntakeState.Retracted;
                }
            }
            double intakeSlidePos = (motors.leftIntakeSlide.getCurrentPosition() + motors.rightIntakeSlide.getCurrentPosition()) * 0.5;
            double error = (intakeTarget - intakeSlidePos);
            double intakeSlideResponse = pidSettings.intakeSlideController.calculate(intakeSlidePos, intakeTarget);
            if (intakeTarget < 5 && intakeSlidePos <= 10) { intakeSlideResponse = 0.0; }
            if (Math.abs(error) < 2) { intakeSlideResponse = 0.0; }
            if (error < 0) {
                if (intakeSlideResponse > 0.3) {
                    intakeSlideResponse = 0.3;
                }
            }
            motors.powers.leftIntakeSlide = intakeSlideResponse;
            motors.powers.rightIntakeSlide = intakeSlideResponse;
        }

        public double newYawCorrection() {
            double rawError = imu.targetYaw - imu.getYawDegrees();
            if (rawError <= -180.0) { rawError += 360.0; }
            if (rawError > 180.0) { rawError -= 360.0; }
            pidSettings.yawController.setPID(pidSettings.yaw_p, pidSettings.yaw_i, pidSettings.yaw_d);
            double response = pidSettings.yawController.calculate(rawError, 0.0);
            return response;
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
        public boolean calculateMovement(GamepadEx gamepad) { // true -> STOP false -> CONTINUE

            // Track number of frames each control has been pressed, made for toggles.
            controller.updateKeyTracker(gamepad);
            double mx = controller.movement_x(gamepad);
            double my = controller.movement_y(gamepad);
            double controllerR = controller.yawRotation(gamepad);
            double controllerR2 = controller.pitchRotation(gamepad);

            imu.targetYaw -= controllerR2 * 8.0; // TODO: Fix joystick axis using Y instead of X

            // Wrap target rotation.
            if (imu.targetYaw < -180) { imu.targetYaw += 360; }
            if (imu.targetYaw > 180) { imu.targetYaw -= 360; }

            // Toggle the intake rollers.
            if (controller.xPress == 1.0) {
                if (state.intake.intakeState == IntakeState.Collecting) {
                    state.intake.intakeState = IntakeState.Retracted;
                } else {
                    state.intake.toggle();
                }
            }

            if (controller.uPress >= 1) {
                mx = 0.0;
                my = -0.3;
            }

            if (controller.rPress >= 1) {
                mx = 0.5;
                my = 0.0;
            }

            if (controller.lPress >= 1) {
                mx = -0.5;
                my = 0.0;
            }

            if (controller.yPress == 1 && state.intake.intakeState == IntakeState.Depositing) {
                state.intake.intakeState = IntakeState.Dropping;
                state.outtake.outtakeState = OuttakeState.Passthrough;
            }

            telemetry.addData("C: green", sensors.g());
            telemetry.addData("A: blue", sensors.b());
            telemetry.addData("B: red", sensors.r());
            telemetry.addData("dist", sensors.intakeColorSensor.getDistance(DistanceUnit.MM));
            telemetry.update();


            // Toggle OUTTAKE state.
            if (controller.aPress == 1.0) { state.outtake.toggle(); }
            componentDrive(my, mx);

            // EMERGENCY STOP
            return (controller.bPress > 0);
        }

        public boolean drive(GamepadEx gamepad) {
            // Calculate drive movement
            if (calculateMovement(gamepad)) {
                return true;
            };
            moveOuttake();
            moveIntake();

            // Update servos / motors
            servos.setPositions(state.outtake.outtakeState, state.intake.intakeState, motors);
            servos.setPowers(state.intake.intakeState, pidSettings.intakeSpeed);
            motors.setPowers();
            return false;
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

        public double oldYawCorrection() {
            double rawError = targetYaw - getYawDegrees();
            if (rawError <= -180.0) { rawError += 360.0; }
            if (rawError > 180.0) { rawError -= 360.0; }
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
