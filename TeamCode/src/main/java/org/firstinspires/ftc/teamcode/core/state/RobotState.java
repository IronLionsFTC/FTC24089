package org.firstinspires.ftc.teamcode.core.state;


public class RobotState {
    public Intake intake = new Intake();
    public Outtake outtake = new Outtake();
    public Drive drive = new Drive();

    public static class Intake {
        public Objects objects = Objects.DEFAULT;
        public Collection collection = Collection.DEFAULT;

        public enum Objects {
            Empty,
            Intermediate,
            Full;

            public static final Objects DEFAULT = Empty;
        }

        public enum Collection {
            Idle,
            Collecting;

            public static final Collection DEFAULT = Idle;
        }
    }

    public static class Outtake {
        public Objects objects = Objects.DEFAULT;
        public Extension extension = Extension.DEFAULT;

        public enum Objects {
            Empty,
            Intermediate,
            Full;

            public static final Objects DEFAULT = Empty;
        }

        public enum Extension {
            Extending,
            Extended,
            Contracting,
            Contracted;

            public static final Extension DEFAULT = Contracted;
        }
    }

    public static class Drive {
        public Movement movement = Movement.DEFAULT;

        public enum Movement {
            Driving,
            Stationary;

            public static final Movement DEFAULT = Stationary;
        }
    }

    public enum Stage {
        TeleOp,
        Autonomous,
        None;

        public static final Stage DEFAULT = None; 
    }
}

