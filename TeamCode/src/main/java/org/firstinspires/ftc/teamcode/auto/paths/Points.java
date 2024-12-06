package org.firstinspires.ftc.teamcode.auto.paths;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class Points {
    public static class SampleStart {
        public static final double basketClose = 3.0;
        public static final Point start = new Point(0.0, 0.0, 1);
        public static final Point submersible = new Point(-26.6, 4.0, 1);
        public static final Point basket = new Point(-6.0 + basketClose, -34.0 - basketClose, 1);
        public static final Point closeBasket = basket;
        public static final Point sampleA = new Point(-8.8, -32.3, 1);
        public static final Point sampleB = new Point(-8.8, -42.2, 1);
        public static final Point sampleC = new Point(-15.7, -35.0, 1);
        public static final Point parkCorner = new Point(-45.0, -34.0, 1);
        public static final Point park = new Point(-49.0, -18.0, 1);
        public static final Point strafeEnd = new Point(-54.0, -15.0, 1);
        public static final Point controlPoint = new Point(-52.0, -38.0, 1);
    }
}
