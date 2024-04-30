package org.firstinspires.ftc.teamcode.Core;

public class Vector_old {
    public double x;
    public double y;
    public double degrees;
    public double magnitude;

    double clamp(double value, double min, double max) {
        if (value < min) return min;
        return Math.min(value, max);
    }

    double compassAtan(double xComponent, double yComponent) {
        // Handle division by zero
        if (y == 0) { return 0; }
        double value = Math.toDegrees(Math.atan(x/y));
        if (x > 0 && y >= 0) { return value;}
        if (x > 0 && y < 0) { return 90 - value;}
        if (x < 0 && y < 0) { return 180 + value;}
        if (x < 0 && y >= 0) {return 360 + value;}
        return 0;
    }

    void fromComponent(double new_x, double new_y) {
        x = new_x; y = new_y;
        magnitude = Math.sqrt(x*x + y*x);
        degrees = compassAtan(x, y);
    }

    void fromPolar(double new_magnitude, double new_degrees) {
        magnitude = new_magnitude; degrees = new_degrees;
        x = Math.sin(Math.toRadians(new_degrees)) * new_magnitude;
        y = Math.cos(Math.toRadians(new_degrees)) * new_magnitude;
    }

    Vector_old add(Vector_old vector) {
        Vector_old newVector = new Vector_old();
        newVector.fromComponent(x + vector.x, y + vector.y);
        return newVector;
    }

    Vector_old subtract(Vector_old vector) {
        Vector_old newVector = new Vector_old();
        newVector.fromComponent(x - vector.x, y - vector.y);
        return newVector;
    }

    Vector_old normalize() {
        Vector_old newVector = new Vector_old();
        newVector.fromComponent(x / magnitude, y / magnitude);
        return newVector;
    }
}
