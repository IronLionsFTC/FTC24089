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
    public QuadrilateralTracker sample = new QuadrilateralTracker();
    List<Vec2> sampling = new ArrayList<>();

    public ComputerVision(HardwareMap hardwareMap, Team colour) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        if (colour == Team.Red) limelight.pipelineSwitch(0); // Red and yellow
        if (colour == Team.Blue) limelight.pipelineSwitch(1); // Blue and yellow
    }

    public void start() {
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    public LLResult analyse() {
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            List<Vec2> corners = getSampleCornerPositions(result);
            if (corners != null) {
                sample.update(corners);
            }
        }
        return result;
    }

    public Vec2 getSamplePosition(LLResult analysis) {
        // Averages the position over several frames to reduce jitter
        if (analysis != null) {
            Vec2 position = new Vec2(analysis.getTx(), analysis.getTy());
            sampling.add(position);
            if (sampling.size() > RobotParameters.Thresholds.CVSmoothing) {
                sampling.remove(0);
            }
            Vec2 average = new Vec2(0.0, 0.0);
            for (Vec2 sample : sampling) {
                average.add(sample);
            }
            average.divide(RobotParameters.Thresholds.CVSmoothing);
            // Account for the camera being rotated 180 degrees to fit the camera cable
            if (sampling.size() >= RobotParameters.Thresholds.CVSmoothing) return new Vec2(average.x * -1, average.y * -1);
            else return null;
        } else {
            sampling.clear();
        }
        return null;
    }

    // Failure - always returns 0.0
    public Double sampleYaw() {
        LLResult analysis = analyse();
        if (analysis == null) return null;
        List<LLResultTypes.ColorResult> crs = analysis.getColorResults();
        for (LLResultTypes.ColorResult cr : crs) {
            Pose3D samplePos = cr.getTargetPoseCameraSpace();
            return samplePos.getOrientation().getYaw(AngleUnit.DEGREES);
        }
        return null;
    }

    public List<Vec2> getSampleCornerPositions(LLResult analysis) {
        List<Vec2> corners = new ArrayList<>();
        if (analysis == null) return null;
        List<LLResultTypes.ColorResult> crs = analysis.getColorResults();
        if (crs.isEmpty()) return null;
        for (List<Double> positions: crs.get(0).getTargetCorners()) {
            corners.add(new Vec2(positions.get(0), positions.get(1)));
        }
        return corners;
    }

    public String getRawOrientation() {
        LLResult analysis = analyse();
        if (analysis == null) return null;
        List<LLResultTypes.ColorResult> crs = analysis.getColorResults();
        if (crs.isEmpty()) return null;
        return crs.get(0).getTargetPoseCameraSpace().getPosition().toString();
    }

    public String getRawCorners() {
        LLResult analysis = analyse();
        if (analysis == null) return null;
        List<LLResultTypes.ColorResult> crs = analysis.getColorResults();
        if (crs.isEmpty()) return null;
        return crs.get(0).getTargetCorners().toString();
    }

    public int getNumResults() {
        LLResult analysis = analyse();
        if (analysis == null) return -1;
        return analysis.getColorResults().size();
    }
}
