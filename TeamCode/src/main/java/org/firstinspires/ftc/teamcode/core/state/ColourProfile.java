package org.firstinspires.ftc.teamcode.core.state;
import org.firstinspires.ftc.teamcode.core.Sensors;
import org.firstinspires.ftc.teamcode.core.params.RobotParameters;

public class ColourProfile {
    public int redCount   = 0;
    public int yellowCount = 0;
    public int blueCount  = 0;
    public int proxCount  = 0;

    private double frameThreshold = RobotParameters.Thresholds.frameThresh;
    private double colourFilter = RobotParameters.Thresholds.colourFilter;

    public double r;
    public double g;
    public double b;
    public double d;

    public ColourProfile(Sensors sensors) {
        r = sensors.r();
        g = sensors.g();
        b = sensors.b();
        d = sensors.d();
    }

    public void update(Sensors sensors) {
        r = sensors.r();
        g = sensors.g();
        b = sensors.b();
        d = sensors.d();
        if (r > (b + g) * colourFilter) { redCount += 1; }
        else { redCount = 0; }
        if (b > (r + g) * colourFilter) { blueCount += 1; }
        else { blueCount = 0; }
        if (b * 2.0 < (r + g) && g > r * colourFilter) { yellowCount += 1; }
        else { yellowCount = 0; }
        if (d < RobotParameters.Thresholds.intakeSamplePresent) { proxCount += 1; }
    }

    public Colour classify() {
        if (redCount >= frameThreshold && proxCount >= frameThreshold) { return Colour.Red; }
        else if (blueCount >= frameThreshold && proxCount >= frameThreshold) { return Colour.Blue; }
        else if (yellowCount >= frameThreshold && proxCount >= frameThreshold) { return Colour.Yellow; }
        else { return Colour.None; }
    }
}
