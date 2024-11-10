package org.firstinspires.ftc.teamcode.core.state;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Arrays;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.core.Vec2;
import org.firstinspires.ftc.teamcode.core.params.RobotParameters;

public class ComputerVision {
    public Limelight3A limelight;
    List<Vec2> sampling = new ArrayList<>();

    public ComputerVision(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // Red and yellow
    }

    public void start() {
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    public LLResult analyse() {
        return limelight.getLatestResult();
    }

    public Vec2 getSamplePosition() {
        // Averages the position over several frames to reduce jitter
        LLResult analysis = analyse();
        if (analysis != null) {
            Vec2 position = new Vec2();
            position.fromComponent(analysis.getTx(), analysis.getTy());
            sampling.add(position);
            if (sampling.size() > RobotParameters.Thresholds.CVSmoothing) {
                sampling.remove(0);
            }
            Vec2 average = new Vec2();
            for (Vec2 sample : sampling) {
                average.add(sample);
            }
            average.divide(RobotParameters.Thresholds.CVSmoothing);
            if (sampling.size() >= RobotParameters.Thresholds.CVSmoothing) return average;
            else return null;
        } else {
            sampling.clear();
        }
        return null;
    }
}
