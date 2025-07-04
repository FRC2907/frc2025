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
    public static final double kDriverDeadband = 0.08;

    public class drivetrain {
        public static final double TRACK_WIDTH = Units.inchesToMeters(22.15); 
        public static final double WHEEL_BASE = Units.inchesToMeters(24.5); 
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(6);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
        public static final double GEAR_RATIO = 7.31; 
        public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2);
        public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(WHEEL_BASE / 2, - TRACK_WIDTH / 2);
        public static final Translation2d REAR_LEFT_LOCATION = new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2);
        public static final Translation2d REAR_RIGHT_LOCATION = new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2);
        public static final MecanumDriveKinematics DRIVE_KINEMATICS = new MecanumDriveKinematics(
            FRONT_LEFT_LOCATION, FRONT_RIGHT_LOCATION, REAR_LEFT_LOCATION, REAR_RIGHT_LOCATION);
        public static final MotorType MOTOR_TYPE = MotorType.kBrushless;

        public static final double kPositionConversionFactor = WHEEL_CIRCUMFERENCE / GEAR_RATIO; //revolutions to meters
        public static final double kVelocityConversionFactor = kPositionConversionFactor / 60; //rpm to m/s
        public static final double kMaxVelRPM = 4000, // revolutions per minute
                                   kMaxAccelRPM = 4000; //revolutions per minute per second (acceleration)
        public static final double kMaxVelMPS = 5.5, // meters per second
                                   kMaxAccelMPS = 12; // meters per second per second OR meters per second squared (m/s^2)
        public static final double kMaxAngularVelRad = 14 * Math.PI, // radians per second
                                   kMaxAngularAccel = 20 * Math.PI; //radians per second per second OR radians per second squared
        public static final double kAllowedError = 0.005;
        public static final double kRampRate = 0.1;
        
        public static final double kSpinSpeed = kMaxAngularVelRad;
        public static final double kDanceMoveSpeed = 3.0; // m/s

        //ALL PIDF CONSTANTS
        public static final double kflFF = 0.00015, //Front left wheel feedforward
                                   kfrFF = 0.000149, //Front right wheel feedforward
                                   krlFF = 0.00015, //Rear left wheel feedforward
                                   krrFF = 0.000152; //Rear right wheel feedforward
        public static final double kP = 4e-7, //All wheels P constant
                                   kI = 1e-7, //All wheels I constant
                                   kD = 3;    //All wheels D constant
        public static final double kHeadingP = 40,
                                   kHeadingD = 8.5;
        //PathPlanner PID constants
        public static final double kPPP = 15.0, //PathPlanner translational P constant
                                   kPPI = 0.0,    //PathPlanner translational I constant
                                   kPPD = 50.0;  //PathPlanner translational D constant
        public static final double kPPRotP = 5.0, //PathPlanner rotational P constant 15.0
                                   kPPRotI = 0.0,   //PathPlanner rotational I constant
                                   kPPRotD = 100.0;   //PathPlanner rotational D constant
        public static final PPHolonomicDriveController PPDriveController = new PPHolonomicDriveController(
            new PIDConstants(kPPP, kPPI, kPPD),           // Translation PID constants
            new PIDConstants(kPPRotP, kPPRotI, kPPRotD)); // Rotation PID constants

        public static final PathConstraints kPathConstraints = new PathConstraints(
            kMaxVelMPS,
            kMaxAccelRPM,
            kMaxAngularVelRad,
            kMaxAngularAccel,
            NOMINAL_VOLTAGE,
            false
        );
    }

    public class coralManipulator {
        public static final MotorType MOTOR_TYPE = MotorType.kBrushless;

        public static final double kShootSpeed = 0.3; //percentage
        public static final double kStopSpeed = 0; //percentage
        public static final double kProximityBand = 100; //TODO tune
    }

    public class algaeManipulator {
        public static final MotorType MOTOR_TYPE = MotorType.kBrushless;
        public static final double GEAR_RATIO = 36.0;

        public static final double kArmConversionFactor = GEAR_RATIO / Units.degreesToRadians(360.0);
        public static final double kArmDownLimit = Units.degreesToRadians(-45); //TODO tune
        public static final double kArmUpLimit = Units.degreesToRadians(160); //TODO tune
        public static final double kMaxVelocity = Units.degreesToRadians(180), //TODO tune (deg/s)
                                   kMaxAcceleration = Units.degreesToRadians(360); //TODO tune (deg/s^2)
        public static final double kAllowedArmError = Units.degreesToRadians(1.5); //TODO tune
        public static final double kAlgaeToElevatorOffset = Units.inchesToMeters(5);

        public static final double kStowAngle = Units.degreesToRadians(95);//TODO find
        public static final double kGrabAngle = Units.degreesToRadians(70); //TODO find
        public static final double kGroundGrabAngle = Units.degreesToRadians(-45); //TODO find
        public static final double kFixedShootAngle = Units.degreesToRadians(156); //TODO tune
        public static final double kIntakeAngle = Units.degreesToRadians(105); //TODO find
        public static final double kProcessorAngle = Units.degreesToRadians(-40); //TODO find

        public static final double kIntakeSpeed = 3000; //TODO find (rpm)
        public static final double kOuttakeSpeed = 3000; //TODO find (rpm)
        public static final double kGrabSpeed = 100; //TODO find (rpm)
        public static final double kFixedShootSpeed = 2000; //TODO find (rpm)
        public static final double kAllowedShootError = 100;
        public static final double kStopSpeed = 0;

        public static final double kProximityBand = 135; //TODO tune

        public static final double kDistanceA = -0.611; //TODO tune (y=mx+b for shoot linear regression equation for distance)
        public static final double kHeightA = -0.3; //TODO tune (y=mx+b for shoot linear regression equation for height)
        public static final double kShootB = 11.3; //TODO tune (y=mx+b for shoot linear regression equation for the base value)

        public static final double kShootP = 1e-5, //TODO tune
                                   kShootI = 7e-7, //TODO tune
                                   kShootD = 0.0115, //TODO tune
                                   kShootFF = 0.000119; //TODO adjust 0.0010905
        public static final double kArmP = 17.5, //TODO tune
                                   kArmI = 0, //TODO tune
                                   kArmD = 0.0; //TODO tune
        public static final double kS = 0.555, 
                                   kG = 0.35, 
                                   kV = 0.70, //TODO adjust (0.01 if degrees)
                                   kA = 0.01; //TODO adjust (0.00 if degrees)
        public static final Constraints kConstraints = new Constraints(
            kMaxVelocity, kMaxAcceleration
        ); //TODO adjust
    }

    public class elevator {
        public static final MotorType MOTOR_TYPE = MotorType.kBrushless;
        public static final double GEAR_RATIO = 9.0;
        public static final double METERS_PER_ROTATION = Units.inchesToMeters(11);

        public static final double kConversionFactor = METERS_PER_ROTATION / GEAR_RATIO; 
        public static final double kOffset = Units.inchesToMeters(6.273); //TODO find better (m.) (original height off ground)
        public static final double kMinVoltage = -4; //TODO find
        public static final double kMaxVoltage = 4; //TODO find
        public static final double kDownLimit = kOffset;
        public static final double kUpLimit = Units.inchesToMeters(50); //TODO find
        public static final double kAllowedError = Units.inchesToMeters(1); //TODO tune
        public static final double kIntakeOffset = Units.inchesToMeters(8.11); //TODO tune
        public static final double kAlgaeOffset = Units.inchesToMeters(22); //TODO tune

        public static final double kMaxVelocity = Units.inchesToMeters(38), //TODO tune (m/s) (38)
                                   kMaxAcceleration = Units.inchesToMeters(76); //TODO tune (m/s^2) (76)

        public static final double kNeutral = Units.inchesToMeters(7.5); //TODO tune
        public static final double kCoralStation = Units.inchesToMeters(37) - kIntakeOffset; //TODO tune
        public static final double kManualControlFactor = Units.inchesToMeters(0.1); //TODO tune
        public static final double kL1 = FieldElements.ReefHeight.L1.height + Units.inchesToMeters(3); //TODO tune
        public static final double kL2 = FieldElements.ReefHeight.L2.height + Units.inchesToMeters(1); //TODO tune
        public static final double kL3 = FieldElements.ReefHeight.L3.height + Units.inchesToMeters(1); //TODO tune
        public static final double kL4 = FieldElements.ReefHeight.L4.height + Units.inchesToMeters(1); //TODO tune 
        public static final double kA1 = kL2 + Units.inchesToMeters(8) - kAlgaeOffset;
        public static final double kA2 = kL3 + Units.inchesToMeters(8) - kAlgaeOffset;

        public static final double kElevatorDriverDeadband = 0.3;

        public static final double kP = 50, //TODO tune
                                   kI = 0, //TODO tune
                                   kD = 0.5; //TODO tune
        
        public static final double kS = 0.5,
                                   kG = 0.45, //0.31
                                   kV = 6.01, //11.97
                                   kA = 0.00; //0.03 
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