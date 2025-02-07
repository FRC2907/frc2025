package frc.robot.constants;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Control {
    public static final String LIMELIGHT_NAME = "";
    public static final int CURRENT_LIMIT = 40; //amps
    public static final double kDriverDeadband = 0.08;

    public class drivetrain {
        public static final double TRACK_WIDTH = Units.inchesToMeters(19.5); //TODO update
        public static final double WHEEL_BASE = Units.inchesToMeters(20.75); //TODO update
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(6);
        public static final double GEAR_RATIO = 5.95;
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
        public static final double kMaxAngularVelRad = 6 * Math.PI; // radians per second
        public static final double kMaxAngularAccel = 10 * Math.PI; //radians per second per second
        public static final double kAllowedError = 0.005;
        public static final double kRampRate = 0.1;

        public static final double kflFF = 0.000158,
                                   kfrFF = 0.000149,
                                   krlFF = 0.000158420,
                                   krrFF = 0.000156;
        public static final double kP = 4e-7,
                                   kI = 1e-7,
                                   kD = 3;
        public static final double kRotP = 7.5,
                                   kRotI = 0,
                                   kRotD = 0;
        public static final double kPPP = 18.5,
                                   kPPI = 0,
                                   kPPD = 2.5;
    }

    public class coralManipulator {
        public static final MotorType MOTOR_TYPE = MotorType.kBrushed;

        public static final double kShootSpeed = 0.3; //percentage
        public static final double kStopSpeed = 0; //percentage
    }

    public class algaeManipulator {
        public static final MotorType MOTOR_TYPE = MotorType.kBrushless;
        public static final double GEAR_RATIO = 1; //TODO find

        public static final double kArmConversionFactor = GEAR_RATIO / 360;
        public static final double kArmDownLimit = 30; //TODO find
        public static final double kArmUpLimit = 135; //TODO find
        public static final double kIntakeAngle = 90; //TODO find
        public static final double kGroundIntakeAngle = 35; //TODO find
        public static final double kIntakeSpeed = 0.1; //TODO find
        public static final double kFixedShootSpeed = 2000; //TODO find
        public static final double kFixedShootAngle = 135; //TODO find

        public static final double kProximityBand = 500; //TODO tune
    }
}
