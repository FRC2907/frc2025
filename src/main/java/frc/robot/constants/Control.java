package frc.robot.constants;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Control {
    public static final String LIMELIGHT_NAME = "";
    public static final int CURRENT_LIMIT = 40; //amps
    public static final double NOMINAL_VOLTAGE = 12.0;
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

        public static final double kMaxAccelRPM = 4000; //revolutions per minute per second (acceleration)
        public static final double kMaxVelRPM = 4000; // revolutions per minute
        public static final double kMaxAccelMPS = 14; // meters per second per second
        public static final double kMaxVelMPS = 14; // meters per second
        public static final double kMaxAngularVelRad = 4.5 * Math.PI; // radians per second
        public static final double kMaxAngularAccel = 9 * Math.PI; //radians per second per second
        public static final double kAllowedError = 0.005;
        public static final double kRampRate = 0.1;

        //All PIDF constants
        public static final double kflFF = 0.000158, //Front left wheel feedforward
                                   kfrFF = 0.000149, //Front right wheel feedforward
                                   krlFF = 0.000158420, //Rear left wheel feedforward
                                   krrFF = 0.000156; //Rear right wheel feedforward
        public static final double kP = 4e-7, //All wheels P constant
                                   kI = 1e-7, //All wheels I constant
                                   kD = 3;    //All wheels D constant
        public static final double kPPP = 18.5, //PathPlanner translational P constant
                                   kPPI = 0,    //PathPlanner translational I constant
                                   kPPD = 2.5;  //PathPlanner translational D constant
        public static final double kPPRotP = 7.5, //PathPlanner rotational P constant
                                   kPPRotI = 0,   //PathPlanner rotational I constant
                                   kPPRotD = 0;   //PathPlanner rotational D constant
        public static final PPHolonomicDriveController PPDriveController = new PPHolonomicDriveController(
            new PIDConstants(kPPP, kPPI, kPPD),           // Translation PID constants
            new PIDConstants(kPPRotP, kPPRotI, kPPRotD)); // Rotation PID constants

        public static final PathConstraints pathConstraints = new PathConstraints(
            kMaxVelMPS,
            kMaxAccelRPM,
            kMaxAngularVelRad,
            kMaxAngularAccel,
            NOMINAL_VOLTAGE,
            false
        );
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
