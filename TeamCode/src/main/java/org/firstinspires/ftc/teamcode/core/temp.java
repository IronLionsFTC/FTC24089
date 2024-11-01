package org.firstinspires.ftc.teamcode.core;

public class temp {

}
/*
package org.firstinspires.ftc.teamcode.core;

import org.firstinspires.ftc.teamcode.core.params.RobotParameters;
import org.firstinspires.ftc.teamcode.core.state.RobotState;

public class temp {
    public void setPositions(OuttakeState outtakeState, IntakeState intakeState, Motors motors, boolean armUp, boolean retract) {
        double bucketPos = 0.0;
        double armPos = 0.0;

        if (outtakeState == OuttakeState.Down || outtakeState == OuttakeState.Folded) {
            bucketPos = RobotParameters.ServoBounds.bucketOpen;
            if (intakeState == IntakeState.Depositing || intakeState == IntakeState.Dropping) {
                armPos = RobotParameters.ServoBounds.armTransfer;
            } else {
                armPos = RobotParameters.ServoBounds.armDown;
            }
        } else if (outtakeState == OuttakeState.Deposit || outtakeState == OuttakeState.Up || outtakeState == OuttakeState.PassthroughDeposit || outtakeState == OuttakeState.Waiting) {
            if (outtakeState == OuttakeState.Deposit || outtakeState == OuttakeState.PassthroughDeposit) {
                bucketPos = 0.0;
            } else if (motors.leftIntakeSlide.getCurrentPosition() > RobotParameters.SlideBounds.intakeClearance - 10.0 || motors.leftOuttakeSlide.getCurrentPosition() > 300.0) {
                bucketPos = RobotParameters.ServoBounds.bucketClosed;
            } else if (outtakeState != OuttakeState.Waiting){
                bucketPos = RobotParameters.ServoBounds.bucketTransfer;
            }
            if (motors.leftOuttakeSlide.getCurrentPosition() > RobotParameters.SlideBounds.outtakeUp - 100.0) {
                if (outtakeState != OuttakeState.Deposit) {
                    armPos = RobotParameters.ServoBounds.armUp;
                } else {
                    armPos = RobotParameters.ServoBounds.armWait;
                }
            } else if (outtakeState == OuttakeState.Up || outtakeState == OuttakeState.Deposit) {
                armPos = RobotParameters.ServoBounds.armWait;
            }
        }
        if (outtakeState == OuttakeState.PassthroughDeposit) {
            armPos = RobotParameters.ServoBounds.armUp;
        }
        if (armUp) {
            armPos = 0.1;
        }
        if (outtakeState == OuttakeState.Waiting) {
            if (motors.leftIntakeSlide.getCurrentPosition() > RobotParameters.SlideBounds.intakeClearance - 10.0 || retract) {
                armPos = RobotParameters.ServoBounds.armWait;
                bucketPos = RobotParameters.ServoBounds.bucketClosed;
            }
        }

        if (intakeState == IntakeState.Retracted || intakeState == IntakeState.Folded) {
            if (motors.leftIntakeSlide.getCurrentPosition() < 10.0 && motors.leftOuttakeSlide.getCurrentPosition() < RobotParameters.Thresholds.outtakeHeightToRetractIntakeLower && outtakeState != OuttakeState.Up) {
                latchServo.setPosition(RobotParameters.ServoBounds.latchClosed);
            } else {
                latchServo.setPosition(RobotParameters.ServoBounds.latchOpened);
            }
        } else {
            latchServo.setPosition(RobotParameters.ServoBounds.latchOpened);
        }

        if (outtakeState == OuttakeState.LevelOneHang) {
            armPos = RobotParameters.ServoBounds.armDown - 0.05;
        }

        if (intakeState == IntakeState.Depositing || intakeState == IntakeState.Dropping) {
            armPos = RobotParameters.ServoBounds.armTransfer;
        }

        if (outtakeState == OuttakeState.Up) {
            bucketPos = RobotParameters.ServoBounds.bucketClosed;
        }

        // The sample is being transistioned into the outtake arm.
        if (outtakeState == OuttakeState.Waiting) {
        }

        bucketServo.setPosition(bucketPos);
        armServoA.setPosition(armPos);
        armServoB.setPosition(1.0 - armPos);
    }


}
*/