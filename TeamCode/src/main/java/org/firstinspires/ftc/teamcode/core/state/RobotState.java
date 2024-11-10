package org.firstinspires.ftc.teamcode.core.state;
import org.firstinspires.ftc.teamcode.core.state.intake.Intake;
import org.firstinspires.ftc.teamcode.core.state.outtake.Outtake;

public class RobotState {
    public Intake intake;
    public Outtake outtake = new Outtake();
    public RobotState(ComputerVision cv) {
        intake = new Intake(cv);
    }
}

