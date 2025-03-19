package frc.robot.constants;

import edu.wpi.first.wpilibj.I2C.Port;

public class Ports {
    public class drivetrain {
        public static final int FRONT_LEFT = 11;
        public static final int FRONT_RIGHT = 10;
        public static final int REAR_LEFT = 20;
        public static final int REAR_RIGHT = 1;
    }

    public class manipulator {
        public static final int CORAL_SHOOTER = 14;

        public static final int ALGAE_ARM = 2;
        public static final int ALGAE_SHOOT_LEADER = 3;
        public static final int ALGAE_SHOOT_FOLLOWER = 5;

        public static final int TOF_SENSOR = 0;
        public static final Port COLOR_SENSOR = Port.kOnboard;
    }

    public class elevator {
        public static final int LEADER = 6;
        public static final int FOLLOWER = 7;
    }

    public class HID {
        public static final int DRIVER = 0;
        public static final int OPERATOR = 1;
    }
}
