package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import org.firstinspires.ftc.teamcode.core.Motors;

// FTCLib

@TeleOp
public class FieldCentricDrive_old extends LinearOpMode{
    @Override
    public void runOpMode() {
        Motors motors = new Motors(hardwareMap);

        Deadline gamepadRateLimit = new Deadline(500, java.util.concurrent.TimeUnit.MILLISECONDS);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        ));
        imu.initialize(parameters);

        waitForStart();

        while (opModeIsActive()) {
            double lx = -gamepad1.left_stick_x;
            double ly = gamepad1.left_stick_y; // TODO changed to -ve, change back if needed
            double rx = gamepad1.right_stick_x;

            double max = Math.max(Math.abs(lx) + Math.abs(ly) + Math.abs(rx), 1);

            double drivePower = 1 - (0.6 * gamepad1.right_trigger);

            if (gamepadRateLimit.hasExpired() && gamepad1.a) {
                imu.resetYaw();
                gamepadRateLimit.reset();
            }

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double adjustedLx = -ly * Math.sin(heading) + lx * Math.cos(heading);
            double adjustedLy = ly * Math.cos(heading) + lx * Math.sin(heading);

            motors.leftFront.set(((adjustedLy + adjustedLx + rx) / max) * drivePower);
            motors.leftBack.set(((adjustedLy - adjustedLx + rx) / max) * drivePower);
            motors.rightFront.set(((adjustedLy - adjustedLx - rx) / max) * drivePower);
            motors.rightBack.set(((adjustedLy + adjustedLx - rx) / max) * drivePower);
        }
    }
}