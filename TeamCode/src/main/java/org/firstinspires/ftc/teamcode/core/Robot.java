package org.firstinspires.ftc.teamcode.core;
import com.acmerobotics.dashboard.FtcDashboard; import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
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
import org.firstinspires.ftc.teamcode.core.state.Colour;
import org.firstinspires.ftc.teamcode.core.state.ColourProfile;
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
    public ColourProfile intakeColour;

    public Robot(HardwareMap h, Telemetry t, Team colour) {
        hardwareMap = h;
        telemetry = t;

        state = new RobotState();
        controller = new Controller();
        blinkin = new Blinkin(hardwareMap);

        drivetrain = new Drivetrain(hardwareMap);
        imu = new RobotIMU(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        sensors = new Sensors(hardwareMap);
        intakeColour = new ColourProfile(sensors);
        team = colour;
    }
    @Config
    public static class PID_settings {
        public static double intakeSlide_p = 0.05;
        public static double intakeSlide_i = 0.05;
        public static double intakeSlide_d = 0.001;

        public PIDController outtakeSlideController = new PIDController(
                RobotParameters.PIDConstants.intakeSlideP,
                RobotParameters.PIDConstants.intakeSlideI,
                RobotParameters.PIDConstants.intakeSlideD);
        public PIDController intakeSlideController = new PIDController(intakeSlide_d, intakeSlide_i, intakeSlide_d);
        public PIDController yawController = new PIDController(
                RobotParameters.PIDConstants.yawP,
                RobotParameters.PIDConstants.yawI,
                RobotParameters.PIDConstants.yawD);
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
            double r = yawCorrection();
            double movementMultiplier = 0.7;
            if (state.intake.intakeState == IntakeState.Collecting) {
                movementMultiplier = 0.5;
            }
            if (state.outtake.outtakeState == OuttakeState.Up) {
                movementMultiplier = 0.3;
            }
            powerVec2.fromComponent(rightPower, forwardPower);
            // Forwards drive (front two need to be reversed)
            motors.powers.leftFront = ((rightPower - forwardPower) * movementMultiplier + r);
            motors.powers.rightFront = ((-rightPower - forwardPower) * movementMultiplier - r);
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
            double slideTarget = RobotParameters.SlideBounds.outtakeDown;
            if ((state.outtake.outtakeState == OuttakeState.Up || state.outtake.outtakeState == OuttakeState.Deposit) && state.intake.intakeState == IntakeState.Retracted) {
                double intakeSlidePos = (motors.leftIntakeSlide.getCurrentPosition() + motors.rightIntakeSlide.getCurrentPosition()) * 0.5;
                if (intakeSlidePos > RobotParameters.Thresholds.intakeClearanceForOuttakeMovement || outtakeSlidePos > RobotParameters.Thresholds.outtakeMinimumHeightToNotWorryAboutIntake) {
                    slideTarget = RobotParameters.SlideBounds.outtakeUp;
                }
            } else {
                slideTarget = RobotParameters.SlideBounds.outtakeDown;
            }

            double outtakeSlideResponse = pidSettings.outtakeSlideController.calculate(outtakeSlidePos, slideTarget);
            double outtakeSlideFeedForward = Math.cos(Math.toRadians(slideTarget / RobotParameters.PIDConstants.ticksInDegree)) * RobotParameters.PIDConstants.intakeSlideF;
            double outtakeSlidePower = outtakeSlideResponse + outtakeSlideFeedForward;

            // Stop the outtake slides from pulling against hard stop, gives 30 degrees of encoder error freedom
            if (slideTarget <= 10.0 && outtakeSlidePos < 30) { outtakeSlidePower = 0.0; }
            motors.powers.leftOuttakeSlide = outtakeSlidePower;
            motors.powers.rightOuttakeSlide = outtakeSlidePower;
        }

        public void moveIntake() {
            // Constantly check for inputs even if not in evaluation mode
            intakeColour.update(sensors);
            // If there is a sample present in the intake and the intake is spinning, evaluate the colour in this loop cycle.
            if (sensors.d() < RobotParameters.Thresholds.intakeSamplePresent * 1.5 // Give a bit of range before attempting a detection
                    && state.intake.intakeState == IntakeState.Collecting) {
                state.intake.intakeState = IntakeState.Evaluating;
            }
            // Apply PID. TODO: Finalise tune & remove from @Config class
            pidSettings.intakeSlideController.setPID(pidSettings.intakeSlide_p, pidSettings.intakeSlide_i, pidSettings.intakeSlide_d);
            double intakeTarget = RobotParameters.SlideBounds.intakeIn;
            double intakeLift = RobotParameters.ServoBounds.intakeFolded;
            if (state.intake.intakeState == IntakeState.Folded) {
                intakeTarget = RobotParameters.SlideBounds.intakeIn;;
                intakeLift = RobotParameters.ServoBounds.intakeFolded;
            } else if (state.intake.intakeState == IntakeState.Retracted || state.intake.intakeState == IntakeState.Folded) {
                if (state.outtake.outtakeState != OuttakeState.Down) {
                    intakeTarget = RobotParameters.SlideBounds.intakeClearance;
                } else {
                    if ((motors.rightOuttakeSlide.getCurrentPosition() + motors.leftOuttakeSlide.getCurrentPosition()) * 0.5
                            < RobotParameters.Thresholds.outtakeHeightToRetractIntakeLower) {
                        intakeTarget = RobotParameters.SlideBounds.intakeIn;
                    } else {
                        intakeTarget = RobotParameters.SlideBounds.intakeClearance;
                    }
                }
                if (state.outtake.outtakeState == OuttakeState.Up || state.outtake.outtakeState == OuttakeState.Deposit) {
                    if (motors.rightOuttakeSlide.getCurrentPosition() > RobotParameters.Thresholds.outtakeHeightToRetractIntakeUpper
                            && motors.leftOuttakeSlide.getCurrentPosition() > RobotParameters.Thresholds.outtakeHeightToRetractIntakeLower) {
                        intakeTarget = RobotParameters.SlideBounds.intakeIn;
                    }
                }
                intakeLift = RobotParameters.ServoBounds.intakeFolded;
            } else if (state.intake.intakeState == IntakeState.Extended) {
                intakeTarget = RobotParameters.SlideBounds.intakeExtended;
                intakeLift = RobotParameters.ServoBounds.intakeFolded;
            } else if (state.intake.intakeState == IntakeState.Collecting) {
                intakeTarget = RobotParameters.SlideBounds.intakeExtended;
                if (controller.yPress == 0) {
                    intakeLift = RobotParameters.ServoBounds.intakeDown;
                } else {
                    // If pressing y while collecting, lift intake.
                    // - Used in situations with stacked samples
                    // - Clear an obstruction without retracting
                    intakeLift = RobotParameters.ServoBounds.intakeDown + 0.25;
                }
            } else if (state.intake.intakeState == IntakeState.Evaluating) {
                // Stay down ready to continue intaking when checking colour
                intakeTarget = RobotParameters.SlideBounds.intakeExtended;
                intakeLift = RobotParameters.ServoBounds.intakeDown;
            } else if (state.intake.intakeState == IntakeState.Depositing || state.intake.intakeState == IntakeState.Dropping) {
                // Transferring sample from intake -> outtake
                intakeTarget = RobotParameters.SystemsTuning.intakeTransfer;
                intakeLift = RobotParameters.ServoBounds.intakeFolded;
            }

            // Lift the intake to the desired position
            // DO NOT CHANGE - THE 1.0 minus is essential
            servos.leftIntakeLiftServo.setPosition(1.0 - intakeLift);
            servos.rightIntakeLiftServo.setPosition(intakeLift);

            if (state.intake.intakeState == IntakeState.Evaluating) {
                Colour currentColour = intakeColour.classify();
                if ((currentColour == Colour.Red || currentColour == Colour.Yellow) && team == Team.Red) {
                    state.intake.intakeState = IntakeState.Depositing;
                } else if ((currentColour == Colour.Blue || currentColour == Colour.Yellow) && team == Team.Blue) {
                    state.intake.intakeState = IntakeState.Depositing;
                } else {
                    state.intake.intakeState = IntakeState.Collecting;
                }
            }
            // Wait for driver to outtake before dropping sample
            // The outtake will wait for the sample to be dropped
            // and for the intake to be clear before moving
            if (state.intake.intakeState == IntakeState.Depositing) {
                if (state.outtake.outtakeState == OuttakeState.Up) {
                    state.intake.intakeState = IntakeState.Dropping;
                }
            }
            // If currently transferring the sample and the sample is not in the intake
            // Then it must be in the outtake, thus move the intake out of the way
            // which allows the outtake to move
            if (state.intake.intakeState == IntakeState.Dropping) {
                if (sensors.d() > RobotParameters.Thresholds.intakeSamplePresent) {
                    state.intake.intakeState = IntakeState.Retracted;
                }
            }
            // Calculate average slide position
            double intakeSlidePos = (motors.leftIntakeSlide.getCurrentPosition() + motors.rightIntakeSlide.getCurrentPosition()) * 0.5;
            double error = (intakeTarget - intakeSlidePos);
            double intakeSlideResponse = pidSettings.intakeSlideController.calculate(intakeSlidePos, intakeTarget);
            // Cut power when slides are almost retracted so they don't pull against a hard stop
            // from encoder error (e.g. 1 off error)
            if (intakeTarget < 5 && intakeSlidePos <= 10) { intakeSlideResponse = 0.0; }
            if (Math.abs(error) < 2) { intakeSlideResponse = 0.0; }
            // Apply powers to the motors
            motors.powers.leftIntakeSlide = intakeSlideResponse;
            motors.powers.rightIntakeSlide = intakeSlideResponse;
        }

        public double yawCorrection() {
            double rawError = imu.targetYaw - imu.getYawDegrees();
            if (rawError <= -180.0) { rawError += 360.0; }
            if (rawError > 180.0) { rawError -= 360.0; }
            double response = pidSettings.yawController.calculate(rawError, 0.0);
			if (Math.abs(response) < 0.1) {
				return 0.0;
			} else { return response; }
        }

        public boolean calculateMovement(GamepadEx gamepad) { // true -> STOP false -> CONTINUE

            // Track number of frames each control has been pressed, made for toggles.
            controller.updateKeyTracker(gamepad);
            double mx = controller.movement_x(gamepad);
            double my = controller.movement_y(gamepad);
            double controllerR = controller.yawRotation(gamepad);
            double controllerR2 = controller.pitchRotation(gamepad);

            imu.targetYaw -= controllerR2 * 6.0;

            if (Math.abs(controllerR2) < 0.01) {
                if (controller.lastYawWasAnalog) {
                    imu.targetYaw = imu.getYawDegrees();
                    controller.lastYawWasAnalog = false;
                }
            } else {
                controller.lastYawWasAnalog = true;
            }

            // Toggle the intake state
            if (controller.xPress == 1.0) {
                if (state.intake.intakeState == IntakeState.Collecting) {
                    state.intake.intakeState = IntakeState.Retracted;
                } else {
                    state.intake.toggle();
                }
            }

            if (controller.uPress >= 1) {
                mx = 0.0;
                my = -0.4;
            }

            if (controller.rPress >= 1) {
                mx = 0.5;
                my = 0.0;
            }

            if (controller.lPress >= 1) {
                mx = -0.5;
                my = 0.0;
            }

            if (controller.yPress == 1 && state.intake.intakeState == IntakeState.Retracted) {
                imu.targetYaw = 0.0;
                controller.lastYawWasAnalog = false;
            }

            if (controller.yPress == 1 && state.intake.intakeState == IntakeState.Depositing) {
                state.intake.intakeState = IntakeState.Dropping;
                state.outtake.outtakeState = OuttakeState.Passthrough;
            }

            if (controller.yPress == 1 && state.outtake.outtakeState == OuttakeState.Passthrough) {
                state.outtake.outtakeState = OuttakeState.PassthroughDeposit;
            }

            if (controller.lbPress == 1) {
                imu.targetYaw -= 45.0;
                controller.lastYawWasAnalog = false;
            }

            if (controller.rbPress == 1) {
                imu.targetYaw += 45.0;
                controller.lastYawWasAnalog = false;
            }

            // Wrap target rotation
            if (imu.targetYaw < -180) { imu.targetYaw += 360; }
            if (imu.targetYaw > 180) { imu.targetYaw -= 360; }

            // Toggle OUTTAKE state.
            if (controller.aPress == 1.0) { state.outtake.toggle(); }
            componentDrive(my, mx);

            telemetry.update();

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
            servos.intakeOverridePower = controller.right_trigger(gamepad) - controller.left_trigger(gamepad);
            servos.setPositions(state.outtake.outtakeState, state.intake.intakeState, motors, controller.yPress > 0 && state.outtake.outtakeState == OuttakeState.Deposit);
            servos.setPowers(state.intake.intakeState, RobotParameters.PIDConstants.intakeSpeed, sensors, controller.uPress >= 1);
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
    }
}
