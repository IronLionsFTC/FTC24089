package org.firstinspires.ftc.teamcode.core.old_vector_code;

import androidx.annotation.NonNull;

public class Vector {
    public double x;
    public double y;

    // Constructors
    // Raw constructors
    public Vector(double x_, double y_) {
        x = x_;
        y = y_;
    }
    // From component form
    @NonNull
    public static Vector from_components(double x, double y) {
        return new Vector(x, y);
    }
    // From polar form
    @NonNull
    public static Vector from_compass(double magnitude, double direction) {
        double des_x = Math.sin(Math.toRadians(direction)) * magnitude;
        double des_y = Math.cos(Math.toRadians(direction)) * magnitude;
        return new Vector(des_x, des_y);
    }


    // Vector interaction
    // Addition
    public Vector add(Vector other) {
        return Vector.from_components(
                x + other.x,
                y + other.y
        );
    }
    // Subtraction
    public Vector subtract(Vector other) {
        return Vector.from_components(
                x - other.x,
                y - other.y
        );
    }

    // Multiplication
    // Dot product
    public double dot_prod(Vector other) {
        return x * other.x + y * other.y;
    }


    // Unit vector
    public Vector as_unit_vec() {
        return Vector.from_compass(
                1,
                direction()
        );
    }


    // Getters

    public double[] components() {
        return new double[]{x,y};
    }

    public double[] polar_compass() {
        return new double[]{
                magnitude(),
                direction()
        };
    }
    public double magnitude() {
        return Math.sqrt(x*x + y*y);
    }
    public double direction() {
        // No direction
        if (x == 0 && y == 0) {return 0;}
        // Quadrant 1 (top-right)
        else if (x > 0 && y > 0) {
            return Math.toDegrees(Math.atan(x / y));
        }
        // Quadrant 2 (top-left)
        else if (x < 0 && y > 0) {
            return 270 + Math.toDegrees(Math.atan(y / Math.abs(x)));
        }
        // Quadrant 3 (bottom-left)
        else if (x < 0 && y < 0) {
            return 180 + Math.toDegrees(Math.atan(Math.abs(x) / Math.abs(y)));
        }
        // Quadrant 4 (bottom-right)
        else if (x > 0 && y < 0) {
            return 90 + Math.toDegrees(Math.atan(Math.abs(y) / x));
        }
        // Vertical
        else if (x == 0) {
            if (y > 0) {return 0;}        // Up
            else if (y < 0) {return 180;} // Down
        }
        // Horizontal
        else if (y == 0) {
            if (x < 0) {return 270;}     // Left
            else if (x > 0) {return 90;} // Right
        }
        return 0;
    }
}
