package frc.robot.constants;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class Control {
    public static final String LIMELIGHT_NAME = "limelight-casey"; 
    public static final int CURRENT_LIMIT = 40; //amps
    public static final double NOMINAL_VOLTAGE = 12.0;
    public static final double GRAVITY_CONSTANT = -9.80665;
    public static final double kDriverDeadband = 0.08;

    public class drivetrain {
        public static final double TRACK_WIDTH = Units.inchesToMeters(19.5); //TODO update
        public static final double WHEEL_BASE = Units.inchesToMeters(20.75); //TODO update
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(6);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
        public static final double GEAR_RATIO = 5.95;
        public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2);
        public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(WHEEL_BASE / 2, - TRACK_WIDTH / 2);
        public static final Translation2d REAR_LEFT_LOCATION = new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2);
        public static final Translation2d REAR_RIGHT_LOCATION = new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2);
        public static final MecanumDriveKinematics DRIVE_KINEMATICS = new MecanumDriveKinematics(
            FRONT_LEFT_LOCATION, FRONT_RIGHT_LOCATION, REAR_LEFT_LOCATION, REAR_RIGHT_LOCATION);
        public static final MotorType MOTOR_TYPE = MotorType.kBrushless;

        public static final double kPositionConversionFactor = (1 / GEAR_RATIO) * WHEEL_CIRCUMFERENCE; //revolutions to meters
        public static final double kVelocityConversionFactor = kPositionConversionFactor / 60; //rpm to m/s
        public static final double kMaxAccelRPM = 4000; //revolutions per minute per second (acceleration)
        public static final double kMaxVelRPM = 4000; // revolutions per minute
        public static final double kMaxAccelMPS = 14; // meters per second per second OR meters per second squared (m/s^2)
        public static final double kMaxVelMPS = 14; // meters per second
        public static final double kMaxAngularVelRad = 6.9 * Math.PI; // radians per second
        public static final double kMaxAngularAccel = 9 * Math.PI; //radians per second per second OR radians per second squared
        public static final double kAllowedError = 0.005;
        public static final double kRampRate = 0.1;

        //ALL PIDF CONSTANTS
        public static final double kflFF = 0.000158, //Front left wheel feedforward
                                   kfrFF = 0.000149, //Front right wheel feedforward
                                   krlFF = 0.000158420, //Rear left wheel feedforward
                                   krrFF = 0.000156; //Rear right wheel feedforward
        public static final double kP = 4e-7, //All wheels P constant
                                   kI = 1e-7, //All wheels I constant
                                   kD = 3;    //All wheels D constant
        public static final double kHeadingP = 40,
                                   kHeadingD = 8.5;
        //PathPlanner PID constants
        public static final double kPPP = 18.5, //PathPlanner translational P constant
                                   kPPI = 0.0,    //PathPlanner translational I constant
                                   kPPD = 2.5;  //PathPlanner translational D constant
        public static final double kPPRotP = 15.0, //PathPlanner rotational P constant
                                   kPPRotI = 0.0,   //PathPlanner rotational I constant
                                   kPPRotD = 0.0;   //PathPlanner rotational D constant
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
        public static final double kProximityBand = 100; //TODO tune
    }

    public class algaeManipulator {
        public static final MotorType MOTOR_TYPE = MotorType.kBrushless;
        public static final double GEAR_RATIO = 1 / 5.95; //TODO find

        public static final double kArmConversionFactor = GEAR_RATIO * 360.0;
        public static final double kArmDownLimit = 30; //TODO find
        public static final double kArmUpLimit = 135; //TODO find
        public static final double kMaxVelocity = 180, //TODO tune (deg/s)
                                   kMaxAcceleration = 360; //TODO tune (deg/s^2)
        public static final double kAllowedArmError = 1.5; //TODO tune
        public static final double kAlgaeToElevatorOffset = Units.inchesToMeters(5);

        public static final double kStowAngle = 105;//TODO find
        public static final double kGrabAngle = 80; //TODO find
        public static final double kGroundGrabAngle = -45; //TODO find
        public static final double kFixedShootAngle = 135; //TODO find
        public static final double kIntakeAngle = 110; //TODO find
        public static final double kProcessorAngle = -40; //TODO find

        public static final double kIntakeSpeed = 300; //TODO find (rpm)
        public static final double kOuttakeSpeed = 300; //TODO find (rpm)
        public static final double kGrabSpeed = 100; //TODO find (rpm)
        public static final double kFixedShootSpeed = 2000; //TODO find (rpm)
        public static final double kAllowedShootError = 100;
        public static final double kStopSpeed = 0;

        public static final double kProximityBand = 500; //TODO tune

        public static final double kDistanceA = -0.611; //TODO tune (y=mx+b for shoot linear regression equation for distance)
        public static final double kHeightA = -0.3; //TODO tune (y=mx+b for shoot linear regression equation for height)
        public static final double kShootB = 11.3; //TODO tune (y=mx+b for shoot linear regression equation for the base value)

        public static final double kP = 0.11, //TODO tune
                                   kI = 0, //TODO tune
                                   kD = 0; //TODO tune
        public static final double kS = 0.14, //TODO tune
                                   kG = 0.46, //TODO adjust
                                   kV = 0.01, //TODO adjust (0.53)
                                   kA = 0.00; //TODO adjust (0.01)
        public static final Constraints kConstraints = new Constraints(
            kMaxVelocity, kMaxAcceleration
        ); //TODO adjust
    }

    public class elevator {
        public static final MotorType MOTOR_TYPE = MotorType.kBrushless;

        public static final double kConversionFactor = 0.01552; 
        public static final double kOffset = Units.inchesToMeters(5.75); //TODO find better (m.) (original height off ground)
        public static final double kDownLimit = Units.inchesToMeters(6);
        public static final double kUpLimit = Units.inchesToMeters(52); //TODO find
        public static final double kAllowedError = Units.inchesToMeters(1); //TODO tune
        public static final double kIntakeOffset = Units.inchesToMeters(5); //TODO find

        public static final double kMaxVelocity = 6.07, //TODO tune (m/s)
                                   kMaxAcceleration = 10; //TODO tune (m/s^2)

        public static final double kNeutral = Units.inchesToMeters(7.5); //TODO tune
        public static final double kCoralStation = Units.inchesToMeters(28); //TODO find
        public static final double kManualControlFactor = Units.inchesToMeters(3); //TODO tune
        public static final double kL1 = FieldElements.ReefHeight.L1.height + Units.inchesToMeters(1); //TODO tune
        public static final double kL2 = FieldElements.ReefHeight.L2.height + Units.inchesToMeters(1); //TODO tune
        public static final double kL3 = FieldElements.ReefHeight.L3.height + Units.inchesToMeters(1); //TODO tune
        public static final double kL4 = FieldElements.ReefHeight.L4.height + Units.inchesToMeters(1); //TODO tune 


        public static final double kP = 50, //TODO tune
                                   kI = 0, //TODO tune
                                   kD = 0; //TODO tune
        
        public static final double kS = 0.14,
                                   kG = 0.31,
                                   kV = 11.97,
                                   kA = 0.0; //0.03 actual
        /*public static final double kS = 0.12, //TODO tune
                                   kG = 6.01, //TODO adjust
                                   kV = 2.05, //TODO adjust
                                   kA = 0.65; //TODO adjust
        public static final double kS = 0.12,
                                   kG = 0.02,
                                   kV = 18.26,
                                   kA = 0.0;*/
        public static final Constraints kConstraints = new Constraints(
            kMaxVelocity, kMaxAcceleration
        ); //TODO adjust
    }
}
