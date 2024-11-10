package org.firstinspires.ftc.teamcode.core.state;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.Vec2;
import org.firstinspires.ftc.teamcode.core.params.RobotParameters;

import java.util.Arrays;
import java.util.List;
import java.util.ArrayList;

public class QuadrilateralTracker {
    List<Vec2> tl = new ArrayList<>();
    List<Vec2> bl = new ArrayList<>();
    List<Vec2> tr = new ArrayList<>();
    List<Vec2> br = new ArrayList<>();
    public double currentRotation = 0.0;

    public List<Vec2> getSmoothedCorners() {
        Vec2 vtl = new Vec2(0.0, 0.0);
        Vec2 vtr = new Vec2(0.0, 0.0);
        Vec2 vbl = new Vec2(0.0, 0.0);
        Vec2 vbr = new Vec2(0.0, 0.0);

        for (int idx = 0; idx < tl.size(); idx++) {
            vtl = vtl.add(tl.get(idx));
            vtr = vtr.add(tr.get(idx));
            vbl = vbl.add(bl.get(idx));
            vbr = vbr.add(br.get(idx));
        }

        vtl = vtl.divide(RobotParameters.Thresholds.CVSmoothing);
        vtr = vtr.divide(RobotParameters.Thresholds.CVSmoothing);
        vbl = vbl.divide(RobotParameters.Thresholds.CVSmoothing);
        vbr = vbr.divide(RobotParameters.Thresholds.CVSmoothing);

        return Arrays.asList(vtl, vtr, vbl, vbr);
    }

    public double getDirection() {
        List<Vec2> smoothedCorners = getSmoothedCorners();
        double dx = smoothedCorners.get(0).x - smoothedCorners.get(2).x;
        double dy = smoothedCorners.get(0).y - smoothedCorners.get(2).y;
        double dx2 = smoothedCorners.get(0).x - smoothedCorners.get(1).x;
        double dy2 = smoothedCorners.get(0).y - smoothedCorners.get(1).y;
        double mag1 = dx*dx + dy*dy;
        double mag2 = dx2*dx2 + dy2*dy2;
        double new_rot = 0.0;
        if (mag1 > mag2) new_rot = Math.toDegrees(Math.atan2(dx, dy)) / 355.0;
        else new_rot = Math.toDegrees(Math.atan2(dx2, dy2)) / 355.0;
        currentRotation = new_rot;
        return currentRotation;
    }

    public void update(List<Vec2> vertices) {
        if (vertices == null) {
            tl.clear();
            tr.clear();
            bl.clear();
            br.clear();
            return;
        }
        if (vertices.size() < 4) {
            tl.clear();
            tr.clear();
            bl.clear();
            br.clear();
            return;
        }

        Vec2 top = null;
        Vec2 bot = null;
        Vec2 sto = null;
        Vec2 sbo = null;

        for (Vec2 vertex : vertices) {
            if (top == null) {
                top = vertex;
            } else if (sto == null) {
                if (vertex.y >= top.y) {
                    sto = top;
                    top = vertex;
                } else {
                    sto = vertex;
                }
            } else {
                if (vertex.y >= top.y) {
                    sto = top;
                    top = vertex;
                } else if (vertex.y >= sto.y) {
                    sto = vertex;
                }
            }
            if (bot == null) {
                bot = vertex;
            } else if (sbo == null) {
                if (vertex.y <= bot.y) {
                    sbo = bot;
                    bot = vertex;
                } else {
                    sbo = vertex;
                }
            } else {
                if (vertex.y <= bot.y) {
                    sbo = bot;
                    bot = vertex;
                } else if (vertex.y <= sbo.y) {
                    sbo = vertex;
                }
            }
        }
        if (top == null || bot == null || sbo == null || sto == null) return;

        Vec2 vtl = new Vec2(0.0,0.0);
        Vec2 vbr = new Vec2(0.0,0.0);
        Vec2 vbl = new Vec2(0.0,0.0);
        Vec2 vtr = new Vec2(0.0,0.0);

        if (top.x < sto.x) {
            vtl = top;
            vtr = sto;
        } else {
            vtl = sto;
            vtr = top;
        }

        if (bot.x < sbo.x) {
            vbl = bot;
            vbr = sbo;
        } else {
            vbl = sbo;
            vbr = bot;
        }

        tl.add(vtl);
        tr.add(vtr);
        bl.add(vbl);
        br.add(vbr);

        if (tl.size() > RobotParameters.Thresholds.CVSmoothing) tl.remove(0);
        if (tr.size() > RobotParameters.Thresholds.CVSmoothing) tr.remove(0);
        if (bl.size() > RobotParameters.Thresholds.CVSmoothing) bl.remove(0);
        if (br.size() > RobotParameters.Thresholds.CVSmoothing) br.remove(0);
    }
}
