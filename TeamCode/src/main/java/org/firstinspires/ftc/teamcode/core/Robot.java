package org.firstinspires.ftc.teamcode.core;
import com.acmerobotics.dashboard.FtcDashboard; import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.core.control.Controls;
import org.firstinspires.ftc.teamcode.core.params.RobotParameters;
import org.firstinspires.ftc.teamcode.core.state.ComputerVision;
import org.firstinspires.ftc.teamcode.core.state.ProximitySensor;
import org.firstinspires.ftc.teamcode.core.state.RobotState;
import org.firstinspires.ftc.teamcode.core.state.Team;
import org.firstinspires.ftc.teamcode.core.state.intake.IntakeState;
import org.firstinspires.ftc.teamcode.core.state.outtake.OuttakeState;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class Robot {
    public Team team;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public Controls controls;
    public Drivetrain drivetrain;
    public RobotIMU imu;
    public RobotState state;
    public ProximitySensor outtakeColour;
    public PID_settings pidSettings = new PID_settings();
    public ComputerVision computerVision;
    public org.firstinspires.ftc.teamcode.core.state.Orientation orientation;
    public Timer flashTimer = new Timer();
    public Timer outtakePullDown = new Timer();
    private boolean resetActive = false;
    private double cv_rotation = 0.0;

    public Robot(HardwareMap h, Telemetry t, Gamepad g1, Gamepad g2, Team colour) {
        hardwareMap = h;
        telemetry = t;
        computerVision = new ComputerVision(hardwareMap, colour);

        state = new RobotState(computerVision);
        if (g1 != null && g2 != null) {
            controls = new Controls(g1, g2);
        }

        drivetrain = new Drivetrain(hardwareMap);
        imu = new RobotIMU(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        team = colour;
        outtakeColour = new ProximitySensor(hardwareMap);
        orientation = new org.firstinspires.ftc.teamcode.core.state.Orientation(drivetrain.motors);
    }

    public void sleep(int ms) {
        try { Thread.sleep(ms); }
        catch (InterruptedException ignored) {}
    }

    public void blockingRetractAndZeroBothSlides() {
        drivetrain.motors.leftOuttakeSlide.set(-0.2);
        drivetrain.motors.rightOuttakeSlide.set(-0.2);
        drivetrain.motors.intakeSlide.set(-0.5);
        sleep(500);
        drivetrain.motors.intakeSlide.resetEncoder();
        drivetrain.motors.rightOuttakeSlide.resetEncoder();
        drivetrain.motors.leftOuttakeSlide.resetEncoder();
    }

    @Config
    public static class PID_settings {
        public static double intakeSlide_p = 0.1;
        public static double intakeSlide_i = 0.0;
        public static double intakeSlide_d = 0.0;

        public PIDController outtakeSlideController = new PIDController(
                RobotParameters.PIDConstants.outtakeSlideP,
                RobotParameters.PIDConstants.outtakeSlideI,
                RobotParameters.PIDConstants.outtakeSlideD);
        public PIDController intakeSlideController = new PIDController(intakeSlide_p, intakeSlide_i, intakeSlide_d);
        public PIDController yawController = new PIDController(
                RobotParameters.PIDConstants.yawP,
                RobotParameters.PIDConstants.yawI,
                RobotParameters.PIDConstants.yawD);
    }

    public class Drivetrain {
        public Motors motors;
        public Servos servos;
        public PositionTracker PositionTracker;
        // Basically, override yaw correction to stop from over rotating when a joystick rotation is made,
        // do not cut off the yaw correction if the last input was NOT a joystick rotation
        public boolean lastYawWasAnalog = true;

        public Drivetrain(HardwareMap hardwareMap) {
            motors = new Motors(hardwareMap);
            servos = new Servos(hardwareMap);
            imu = new RobotIMU(hardwareMap);
        }

        public class PositionTracker {
            public Vec2 position = new Vec2(0.0,0.0);
            public double fr = 0.0;
            public double fl = 0.0;
            public double br = 0.0;
            public double bl = 0.0;
        }


        public void componentDrive(double forwardPower, double rightPower, Vec2 samplePosition, boolean useCV, double manualYaw) {
            if (samplePosition != null && useCV) {
                double normalisedSampleX = samplePosition.x / 30.0;
                double normalisedSampleY = samplePosition.y / 25.0 + 0.1;
                forwardPower -= normalisedSampleY * 1.5;
                rightPower += normalisedSampleX * 1.5;
            }

            if (manualYaw != 0.0) {
                imu.targetYaw = 0.0;
                orientation.zeroYaw();
                orientation.update();
            }

            if (lastYawWasAnalog) {
                if (orientation.getVelocity() > 10.0) {
                    imu.targetYaw = 0.0;
                    orientation.zeroYaw();
                    orientation.update();
                }
            }

            Double r = yawCorrection() + manualYaw;
            if (r.isNaN()) {
                r = manualYaw;
            }
            telemetry.addData("r", r);
            double movementMultiplier = 1.0;
            if (state.intake.intakeState == IntakeState.ExtendedClawDown || state.intake.intakeState == IntakeState.ExtendedGrabbingOffWallClawOpen) movementMultiplier = 0.4;
            motors.powers.leftFront = ((rightPower - forwardPower) * movementMultiplier + r);
            motors.powers.rightFront = ((-rightPower - forwardPower) * movementMultiplier - r);
            motors.powers.leftBack = ((-rightPower - forwardPower) * movementMultiplier + r);
            motors.powers.rightBack = ((rightPower - forwardPower) * movementMultiplier - r);
        }

        public void moveOuttake(double powerMul, boolean auto, boolean raiseHigher) {
            double outtakeSlidePos = motors.outtakePosition();
            double slideTarget = RobotParameters.SlideBounds.outtakeDown;

            // Automatically lower outtake, auto functions do this separately
            if (!auto && (state.outtake.outtakeState == OuttakeState.UpClawOpen || state.outtake.outtakeState == OuttakeState.UpWaitingToGoDown)) {
                if (state.outtake.outtakeAutomaticFoldDown.getElapsedTimeSeconds() > 0.5) {
                    state.outtake.toggle();
                }
            }

            // Automatically perform transfer, auto functions do this separately
            if (!auto && state.intake.intakeState == IntakeState.Transfer && state.outtake.outtakeState == OuttakeState.DownClawOpen) {
                if (motors.intakePosition() < 15.0 && state.intake.foldIntakeBeforeRetraction.getElapsedTimeSeconds() > 1.0) {
                    state.outtake.outtakeState = OuttakeState.DownClawShut;
                    flashTimer.resetTimer();
                }
            }

            // Automatically fold down when reaches a certain height with specimen to avoid damage
            if (!auto && state.outtake.outtakeState == OuttakeState.UpWithSpecimentGoingDown && state.outtake.outtakeAutomaticFoldDown.getElapsedTimeSeconds() > 0.5) {
                state.outtake.toggle();
            }
            if (state.outtake.outtakeState == OuttakeState.UpWithSpecimenOnBar && !auto) {
                if (state.outtake.outtakeAutomaticFoldDown.getElapsedTimeSeconds() > 1.5 || motors.outtakePosition() > AutonomousRobot.AutonomousTune.outtakeHeightToCancel) {
                    state.outtake.toggle();
                }
            }

            // Automatically flip outtake on raise, auto functions do this separately
            if (!auto && state.outtake.outtakeState == OuttakeState.UpWaitingToFlip && motors.outtakePosition() > RobotParameters.SlideBounds.outtakeUp - 150) {
                state.outtake.toggle();
            }

            if (!auto && state.outtake.outtakeState == OuttakeState.UpWaitingToFlip && state.outtake.useLowBasket && motors.outtakePosition() > RobotParameters.SlideBounds.outtakeLowBasket - 150) {
                state.outtake.toggle();
            }

            // If waiting to flip up do so, auto functions do this separately
            if (!auto && state.outtake.outtakeState == OuttakeState.UpWithSpecimenWaitingToFlip && state.outtake.outtakeAutomaticFoldDown.getElapsedTimeSeconds() > 0.3) {
                state.outtake.toggle();
            }

            if (!auto && state.intake.intakeState == IntakeState.ExtendedClawShut) {
                if (motors.intakePosition() > RobotParameters.SlideBounds.intakeExtended - 50.0) {
                    state.intake.intakeState = IntakeState.Retracted;
                    state.outtake.outtakeAutomaticFoldDown.resetTimer();
                }
            }

            switch (state.outtake.outtakeState) {
                case UpWaitingToFlip: case UpFlipped: case UpWaitingToGoDown: case UpClawOpen:
                    if (state.outtake.useLowBasket) slideTarget = RobotParameters.SlideBounds.outtakeLowBasket;
                    else slideTarget = RobotParameters.SlideBounds.outtakeUp;
                    break;
                case UpWithSpecimenWaitingToFlip: case UpWithSpecimenFlipped: case UpWithSpecimentGoingDown:
                    if (auto && raiseHigher) slideTarget = RobotParameters.SlideBounds.outtakeBelowSpecimenBar + 62;
                    else if (auto) slideTarget = RobotParameters.SlideBounds.outtakeBelowSpecimenBar;
                    else slideTarget = RobotParameters.SlideBounds.outtakeBelowSpecimenBar + 65.0;
                    break;
                case UpWithSpecimenOnBar:
                    slideTarget = RobotParameters.SlideBounds.outtakeOnSpecimenBar;
                    break;
            }

            double outtakeSlideResponse = pidSettings.outtakeSlideController.calculate(outtakeSlidePos, slideTarget);
            double outtakeSlideFeedForward = Math.cos(Math.toRadians(slideTarget / RobotParameters.PIDConstants.ticksInDegree)) * RobotParameters.PIDConstants.outtakeSlideF;
            double outtakeSlidePower = outtakeSlideResponse + outtakeSlideFeedForward;

            // Stop the outtake slides from pulling against hard stop, gives 30 degrees of encoder error freedom
            if (slideTarget < 2.0 && outtakeSlidePos < 5) { outtakeSlidePower = 0.0; }
            if (outtakeSlideResponse < 0.0 && outtakeSlidePos > 600.0) { outtakeSlidePower = 0.0; }
            if (state.outtake.outtakeState == OuttakeState.UpWithSpecimenOnBar) { outtakeSlidePower *= 2.0; }
            if (state.outtake.outtakeState == OuttakeState.UpWithSpecimentGoingDown) { outtakeSlidePower = 0.0; }
            if (state.outtake.outtakeState == OuttakeState.DownClawOpen && motors.outtakePosition() > 10) {
                outtakeSlidePower = -0.5;
            }
            motors.powers.leftOuttakeSlide = outtakeSlidePower * powerMul;
            motors.powers.rightOuttakeSlide = outtakeSlidePower * powerMul;
        }

        public void moveIntake(double multiplier, boolean auto) {
            // Automatically let go of intake claw when transferring in teleop
            if (!auto && state.outtake.outtakeState != OuttakeState.DownClawOpen && state.outtake.outtakeState != OuttakeState.DownClawShut && state.intake.intakeState == IntakeState.Transfer) {
                state.intake.intakeState = IntakeState.Retracted;
            }

            double intakeTarget = RobotParameters.SlideBounds.intakeIn;
            switch (state.intake.intakeState) {
                case ExtendedClawUp: case ExtendedClawDown: case Grabbing: case ExtendedGrabbingOffWallClawShut: case ExtendedGrabbingOffWallClawOpen: case ExtendedClawShut: case ExtendedClawOpen:
                    intakeTarget = RobotParameters.SlideBounds.intakeExtended;
                    state.outtake.outtakeState = OuttakeState.DownClawOpen;
            }

            // Delay slide retraction until intake has folded. Teleop & Auto
            if (state.intake.intakeState == IntakeState.Transfer && state.intake.foldIntakeBeforeRetraction.getElapsedTime() < 500) {
                intakeTarget = RobotParameters.SlideBounds.intakeExtended;
                motors.powers.leftOuttakeSlide = -0.2;
                motors.powers.rightOuttakeSlide = -0.2;
            }

            double intakeSlidePos = motors.intakePosition();
            double intakeSlideResponse = pidSettings.intakeSlideController.calculate(intakeSlidePos, intakeTarget) * multiplier;
            if (intakeTarget < 5 && intakeSlidePos <= 15) { intakeSlideResponse = 0.0; }

            // Let go of sample with intake when outtake has grabbed
            // if (!auto && state.intake.intakeState == IntakeState.Transfer && state.outtake.outtakeState == OuttakeState.DownClawShut) {
              //   state.intake.toggle();
            // }

            if (state.intake.intakeState == IntakeState.Retracted || state.intake.intakeState == IntakeState.Transfer) {
                if (motors.intakePosition() < 15.0) {
                    intakeSlideResponse = 0.0;
                }
            } else {
                if (motors.intakePosition() > 100.0) {
                    intakeSlideResponse = 0.2;
                }
            }

            motors.powers.leftIntakeSlide = intakeSlideResponse;
            motors.powers.rightIntakeSlide = intakeSlideResponse;
        }

        public double yawCorrection() {
            double rawError = imu.targetYaw - orientation.getYaw();
            if (rawError <= -180.0) { rawError += 360.0; }
            if (rawError > 180.0) { rawError -= 360.0; }
            double response = pidSettings.yawController.calculate(rawError, 0.0);
			if (Math.abs(response) < 0.1) {
				return 0.0;
			} else { return response; }
        }

        public boolean calculateMovement(GamepadEx gamepad, Vec2 samplePosition) {
            // true -> STOP false -> CONTINUE

            // Track number of frames each control has been pressed, made for toggles.
            controls.update();

            double mx = controls.movement.X();
            double my = controls.movement.Y();
            double yaw = controls.movement.yaw();
            double pitch = controls.movement.pitch();

            // imu.targetYaw -= yaw * 6.0;

            if (Math.abs(yaw) < 0.01) {
                if (lastYawWasAnalog) {
                    imu.targetYaw = orientation.getYaw();
                    lastYawWasAnalog = false;
                }
            } else {
                lastYawWasAnalog = true;
            }

            if (state.intake.intakeState == IntakeState.Transfer && controls.intake.claw.deposit()) {
                state.outtake.outtakeAutomaticFoldDown.resetTimer();
                state.intake.intakeState = IntakeState.ExtendedClawShut;
            }

            // Toggle the intake state
            if (state.intake.intakeState == IntakeState.ExtendedClawUp && controls.util_button_press()) {
                state.intake.intakeState = IntakeState.ExtendedGrabbingOffWallClawOpen;
            } else if (controls.intake.toggle_state()) {
                state.intake.toggle();
            }

            if (state.outtake.outtakeState == OuttakeState.DownClawShut && controls.util_button_press()) {
                state.outtake.outtakeState = OuttakeState.UpWithSpecimenWaitingToFlip;
                state.outtake.outtakeAutomaticFoldDown.resetTimer();
            } else if (controls.outtake.toggle_state()) {
                state.outtake.toggle();
            }

            if (controls.outtake.reset()) {
                state.outtake.outtakeState = OuttakeState.DownClawOpen;
                motors.leftOuttakeSlide.resetEncoder();
                motors.rightOuttakeSlide.resetEncoder();
            }

            if (controls.RESET()) {
                state.intake.intakeState = IntakeState.Retracted;
                motors.intakeSlide.resetEncoder();
                motors.powers.leftIntakeSlide = -1.0;
                motors.powers.rightIntakeSlide = -1.0;
            }

            if (controls.movement.CW45()) {
                imu.targetYaw -= 45.0;
                lastYawWasAnalog = false;
            }

            if (controls.movement.CCW45()) {
                imu.targetYaw += 45.0;
                lastYawWasAnalog = false;
            }

            if (controls.intake.claw.emergencyReset()) {
                if (!resetActive) {
                    resetActive = true;
                    servos.intakeLiftServo.kill();
                    servos.intakeClawServo.kill();
                    servos.outtakeClawServo.kill();
                    servos.intakeYawServo.kill();
                    state.intake.intakeState = IntakeState.ExtendedClawDown;
                } else {
                    resetActive = false;
                    servos.intakeYawServo.revive();
                    servos.outtakeClawServo.revive();
                    servos.intakeClawServo.revive();
                    servos.intakeLiftServo.revive();
                }
            }

            // Wrap target rotation
            while (imu.targetYaw < -180) { imu.targetYaw += 360; }
            while (imu.targetYaw > 180) { imu.targetYaw -= 360; }

            // If grabbing, y releases claw in case of miss.
            if (controls.util_button_press()) {
                if (controls.use_cv() && state.intake.intakeState == IntakeState.Grabbing) {
                    computerVision.start();
                    computerVision.sample.currentRotation = 0.0;
                    state.intake.intakeState = IntakeState.ExtendedClawUp;
                }
            }

            if (controls.RESETYAW()) {
                imu.resetYaw();
                imu.targetYaw = 0.0;
            }

            if (controls.outtake.swapBasket()) {
                state.outtake.useLowBasket = !state.outtake.useLowBasket;
            }

            componentDrive(my, mx, samplePosition, controls.use_cv(), yaw);

            state.intake.clawYaw -= (controls.intake.claw.CW_rotation() - controls.intake.claw.CCW_rotation()) * 0.02;

            if (controls.outtake.pullDown()) {
                motors.powers.leftOuttakeSlide = -0.5;
                motors.powers.rightOuttakeSlide = -0.5;
            }

            // EMERGENCY STOP
            return controls.EMERGENCY_STOP();
        }

        public void update_teleop() {
            moveOuttake(1.0, false, false);
            moveIntake(1.0, false);

            // DEPRECATED | will be removed soon
            // servos.intakeOverridePower = controller.RT() - controller.LT();

            telemetry.addData("dist", outtakeColour.getDistance());
            telemetry.addData("LED", outtakeColour.getLED());
            telemetry.update();
        }

        public boolean drive(GamepadEx gamepad) {
            orientation.update();
            Vec2 samplePosition = null;
            if (state.intake.intakeState == IntakeState.ExtendedClawDown) {
                samplePosition = computerVision.getSamplePosition(computerVision.analyse());
                if (computerVision.sample.currentRotation != 0.0) cv_rotation = computerVision.sample.currentRotation;
                if (samplePosition != null) {
                    if (controls.use_cv()) {
                        LLResult analysis = computerVision.analyse();
                        if (analysis != null) {
                            computerVision.sample.update(computerVision.getSampleCornerPositions(analysis));
                            computerVision.sample.getDirection();
                        }
                    }
                }
            } else {
                cv_rotation = 0.0;
            }
            telemetry.update();

            // Calculate drive movement
            this.update_teleop(); // Cv rotation | disabled so I can use in teleop.
            if (calculateMovement(gamepad, samplePosition)) return true;
            motors.setOtherPowers();
            motors.setDrivePowers();
            servos.setPositions(state.outtake.outtakeState, state.intake.intakeState, motors, state.intake.clawYaw, 0.0, false, controls.RESET());
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
            imu.resetDeviceConfigurationForOpMode();
        }

        public double getYawDegrees() {
            Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if (angles == null) return -0.00001;
            return AngleUnit.DEGREES.normalize(angles.firstAngle);
        }
    }
}
