package org.firstinspires.ftc.teamcode.core.state;


public class RobotState {
    public Intake intake = new Intake();

    public static class Intake {
        public Objects objects = OBJECTS_PRESTART;
        public Collection collection = COLLECTION_PRESTART;

        static Objects OBJECTS_PRESTART = Objects.Empty;
        public enum Objects {
            Empty,
            Intermediate,
            Full
        }

        static Collection COLLECTION_PRESTART = Collection.Idle;
        public enum Collection {
            Idle,
            Collecting
        }
    }
}

