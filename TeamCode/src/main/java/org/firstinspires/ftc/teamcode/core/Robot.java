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

    public void prepareTransferForPreloadedSample() {
        state.intake.intakeState = IntakeState.Depositing;
    }

    public void extendIntakeFromFoldedPosition() {
        state.intake.intakeState = IntakeState.Extended;
    }

    public boolean isIntakeExtended() {
        return state.intake.intakeState == IntakeState.Extended && drivetrain.motors.intakePosition() > RobotParameters.SlideBounds.intakeExtended - 10.0;
    }

    public void foldDownIntakeAndStartCollecting() {
        state.intake.intakeState = IntakeState.Collecting;
    }

    public boolean tryToCollectSample() {
        return drivetrain.moveIntake();
    }

    public boolean tryTransfer() {
        if (state.intake.intakeState == IntakeState.Depositing) {
            // If the slides are within 5 degrees of the target position for transfer.
            if (Math.abs(drivetrain.motors.intakePosition() - RobotParameters.SlideBounds.intakeTransfer) < 5.0) {
                // Outtake will wait until intake has performed transfer before going up, hence 'waiting'
                state.intake.intakeState = IntakeState.Dropping;
                state.outtake.outtakeState = OuttakeState.Waiting;
                return true;
            }
        }
        return false;
    }

    // Check if the transfer is done, then retract slides & return true to move on to next stage
    public boolean transferCompleted() {
        // If the intake is retracted but extended for the arm to move up, then transfer is done.
        if (state.intake.intakeState == IntakeState.Retracted && drivetrain.motors.intakePosition() > RobotParameters.SlideBounds.intakeClearance - 10.0) {
            if (state.outtake.outtakeState == OuttakeState.Waiting) {
                state.outtake.retract = true;
                return true;
            }
        }
        return false;
    }

    public void extendOuttakeToTop() {
        if (state.outtake.outtakeState == OuttakeState.Waiting) {
            state.outtake.outtakeState = OuttakeState.Up;
        }
    }

    public boolean areSlidesReady() {
        return state.outtake.outtakeState == OuttakeState.Up && drivetrain.motors.outtakePosition() > RobotParameters.SlideBounds.outtakeUp - 200.0;
    }

    public void dropSample() {
        if (state.outtake.outtakeState == OuttakeState.Up) state.outtake.outtakeState = OuttakeState.Deposit;
        update_auto();
    }

    public void lowerSlides() {
        if (state.outtake.outtakeState == OuttakeState.Deposit) state.outtake.outtakeState = OuttakeState.Down;
        state.outtake.retract = false;
        state.intake.intakeState = IntakeState.Extended;
    }

    public boolean areSlidesDown() {
        boolean ret = state.outtake.outtakeState == OuttakeState.Down && state.intake.intakeState == IntakeState.Extended && drivetrain.motors.outtakePosition() < 100.0;
        if (ret) {
            state.intake.intakeState = IntakeState.Retracted;
        }
        return ret;
    }

    public boolean areSlidesRetracted() {
        state.intake.intakeState = IntakeState.Retracted;
        return drivetrain.motors.intakePosition() < 5.0;
    }

    public void update_auto() {
        drivetrain.moveIntake();
        drivetrain.moveOuttake();
        drivetrain.motors.setOtherPowers();
        drivetrain.servos.setPositions(state, drivetrain.motors);
        drivetrain.servos.setPowers(state.intake.intakeState, 0.0, sensors, false);
    }

    @Config
    public static class PID_settings {
        public static double intakeSlide_p = 0.02;
        public static double intakeSlide_i = 0.0;
        public static double intakeSlide_d = 0.002;

        public PIDController outtakeSlideController = new PIDController(
                RobotParameters.PIDConstants.outtakeSlideP,
                RobotParameters.PIDConstants.outtakeSlideI,
                RobotParameters.PIDConstants.outtakeSlideD);
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


        public void componentDrive(double forwardPower, double rightPower) {
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
            double outtakeSlidePos = motors.outtakePosition();
            double slideTarget = RobotParameters.SlideBounds.outtakeDown;
            if ((state.outtake.outtakeState == OuttakeState.Up || state.outtake.outtakeState == OuttakeState.Deposit) && state.intake.intakeState == IntakeState.Retracted) {
                double intakeSlidePos = motors.intakePosition();
                if (intakeSlidePos > RobotParameters.Thresholds.intakeClearanceForOuttakeMovement || outtakeSlidePos > RobotParameters.Thresholds.outtakeMinimumHeightToNotWorryAboutIntake || state.outtake.retract) {
                    slideTarget = RobotParameters.SlideBounds.outtakeUp;
                }
            }

            double outtakeSlideResponse = pidSettings.outtakeSlideController.calculate(outtakeSlidePos, slideTarget);
            double outtakeSlideFeedForward = Math.cos(Math.toRadians(slideTarget / RobotParameters.PIDConstants.ticksInDegree)) * RobotParameters.PIDConstants.outtakeSlideF;
            double outtakeSlidePower = outtakeSlideResponse + outtakeSlideFeedForward;

            // Stop the outtake slides from pulling against hard stop, gives 30 degrees of encoder error freedom
            if (slideTarget <= 10.0 && outtakeSlidePos < 30) { outtakeSlidePower = 0.0; }
            motors.powers.leftOuttakeSlide = outtakeSlidePower;
            motors.powers.rightOuttakeSlide = outtakeSlidePower;
        }

        public boolean moveIntake() {
            boolean didCollect = false;
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
                    if (motors.outtakePosition() < RobotParameters.Thresholds.outtakeHeightToRetractIntakeLower) {
                        intakeTarget = RobotParameters.SlideBounds.intakeIn;
                    } else {
                        intakeTarget = RobotParameters.SlideBounds.intakeClearance;
                    }
                }
                if (state.outtake.outtakeState == OuttakeState.Up || state.outtake.outtakeState == OuttakeState.Deposit || (state.outtake.outtakeState == OuttakeState.Waiting && state.outtake.retract)) {
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

            if (state.outtake.retract && state.intake.intakeState == IntakeState.Retracted) {
                intakeTarget = RobotParameters.SlideBounds.intakeIn;
            }

            // Lift the intake to the desired position
            // DO NOT CHANGE - THE 1.0 minus is essential
            servos.leftIntakeLiftServo.setPosition(1.0 - intakeLift);
            servos.rightIntakeLiftServo.setPosition(intakeLift);

            if (state.intake.intakeState == IntakeState.Evaluating) {
                Colour currentColour = intakeColour.classify();
                if ((currentColour == Colour.Red || currentColour == Colour.Yellow) && team == Team.Red) {
                    didCollect = true;
                    state.intake.intakeState = IntakeState.Depositing;
                } else if ((currentColour == Colour.Blue || currentColour == Colour.Yellow) && team == Team.Blue) {
                    state.intake.intakeState = IntakeState.Depositing;
                    didCollect = true;
                } else {
                    state.intake.intakeState = IntakeState.Collecting;
                }
            }
            // Wait for driver to outtake before dropping sample
            // The outtake will wait for the sample to be dropped
            // and for the intake to be clear before moving
            if (state.intake.intakeState == IntakeState.Depositing) {
                if (state.outtake.outtakeState == OuttakeState.Waiting) {
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

            if (state.outtake.outtakeState == OuttakeState.Waiting && state.outtake.retract) {
                intakeTarget = RobotParameters.SlideBounds.intakeIn;
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

            return didCollect || state.intake.intakeState == IntakeState.Depositing;
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

        public boolean calculateMovement(GamepadEx gamepad) {
            // true -> STOP false -> CONTINUE

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
            if (controller.xPress == 1.0 && state.outtake.outtakeState == OuttakeState.Waiting) {
                state.outtake.retract = true;
                state.intake.intakeState = IntakeState.Retracted;
            }
            componentDrive(my, mx);

            telemetry.addData("IS POS", motors.intakePosition());
            telemetry.addData("OS POS", motors.outtakePosition());

            telemetry.update();

            if (state.outtake.outtakeState == OuttakeState.Down && motors.outtakePosition() < 50.0) {
                motors.leftOuttakeSlide.resetEncoder();
                motors.rightOuttakeSlide.resetEncoder();
            }

            // EMERGENCY STOP
            return (controller.bPress > 0);
        }

        public void update_teleop(GamepadEx gamepad) {
            moveOuttake();
            moveIntake();

            // Update servos / motors
            servos.intakeOverridePower = controller.right_trigger(gamepad) - controller.left_trigger(gamepad);
            servos.setPositions(state, motors);
            servos.setPowers(state.intake.intakeState, RobotParameters.PIDConstants.intakeSpeed, sensors, controller.uPress >= 1);
            motors.setDrivePowers();
            motors.setOtherPowers();
        }

        public boolean drive(GamepadEx gamepad) {
            // Calculate drive movement
            if (calculateMovement(gamepad)) {
                return true;
            };
            this.update_teleop(gamepad);

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
