package frc.robot.constants;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Control {
    public static final double kDriverDeadband = 0.08;
    public static final int kCurrentLimit = 40; //amps

    public class drivetrain {
        public static final double TRACK_WIDTH = Units.inchesToMeters(19.5); //TODO update
        public static final double WHEEL_BASE = Units.inchesToMeters(20.75); //TODO update
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(6);
        public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2);
        public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(WHEEL_BASE / 2, - TRACK_WIDTH / 2);
        public static final Translation2d REAR_LEFT_LOCATION = new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2);
        public static final Translation2d REAR_RIGHT_LOCATION = new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2);
        public static final MecanumDriveKinematics DRIVE_KINEMATICS = new MecanumDriveKinematics(
            FRONT_LEFT_LOCATION, FRONT_RIGHT_LOCATION, REAR_LEFT_LOCATION, REAR_RIGHT_LOCATION);
        public static final MotorType MOTOR_TYPE = MotorType.kBrushless;

        public static final double kMaxAcceleration = 4000; //rpm / s
        public static final double kMaxVelRPM = 4000; // rpm
        public static final double kMaxVelMeters = 14; // m/s
        public static final double kMaxAngularVel = 14;
        public static final double kAllowedError = 0.005;
        public static final double kRampRate = 0.1;

        public static final double kflFF = 0.000158,
                                   kfrFF = 0.000149,
                                   krlFF = 0.000158420,
                                   krrFF = 0.000156;
        public static final double kP = 4e-7,
                                   kI = 1e-7,
                                   kD = 3;
    }

    public class coralManipulator {
        public static final MotorType MOTOR_TYPE = MotorType.kBrushed;

        public static final double kShootSpeed = 0.3;
    }

    public class algaeManipulator {
        public static final MotorType MOTOR_TYPE = MotorType.kBrushless;
        public static final double GEAR_RATIO = 1; //TODO find

        public static final double armConversionFactor = GEAR_RATIO / 360;
        public static final double armDownLimit = 30; //TODO find
        public static final double armUpLimit = 135; //TODO find
        public static final double shootAngle = 100; //TODO find
    }
}
