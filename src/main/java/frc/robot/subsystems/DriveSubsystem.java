// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Stack;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Util;
import frc.robot.constants.Control;
import frc.robot.constants.FieldConstraints;
import frc.robot.constants.Ports;

public class DriveSubsystem extends SubsystemBase {

  private final Field2d field;

  private double frontLeftSpeed, frontRightSpeed, rearLeftSpeed, rearRightSpeed;
  private double wheelDiameter;
  private SparkMax frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor;
  private SparkMaxConfig config;
  private RelativeEncoder frontLeftEnc, frontRightEnc, rearLeftEnc, rearRightEnc;
  private MecanumDrive dt;
  private AHRS gyro;
  private MecanumDrivePoseEstimator poseEstimator;
  private MecanumDriveKinematics kinematics;
  private RobotConfig robotConfig;
  private String limelight;
  private LimelightHelpers.PoseEstimate limelightMeasurement;
  //private HolonomicDriveController driveController;

  public DriveSubsystem() {
    frontLeftMotor =  new SparkMax(Ports.drivetrain.FRONT_LEFT,  MotorType.kBrushless);
    frontRightMotor = new SparkMax(Ports.drivetrain.FRONT_RIGHT, MotorType.kBrushless);
    rearLeftMotor =   new SparkMax(Ports.drivetrain.REAR_LEFT,   MotorType.kBrushless);
    rearRightMotor =  new SparkMax(Ports.drivetrain.REAR_RIGHT,  MotorType.kBrushless);

    frontLeftEnc =  frontLeftMotor.getEncoder();
    frontRightEnc = frontRightMotor.getEncoder();
    rearLeftEnc =   rearLeftMotor.getEncoder();
    rearRightEnc =  rearRightMotor.getEncoder();

    frontLeftEnc.setPosition(0);
    frontRightEnc.setPosition(0);
    rearLeftEnc.setPosition(0);
    rearRightEnc.setPosition(0);

    frontLeftSpeed = 0;
    frontRightSpeed = 0;
    rearLeftSpeed = 0;
    rearRightSpeed = 0;

    wheelDiameter = Control.drivetrain.WHEEL_DIAMETER;

    configure();

    //dt = new MecanumDrive(frontLeftMotor::set, rearLeftMotor::set, frontRightMotor::set, rearRightMotor::set);

    gyro = new AHRS(NavXComType.kMXP_SPI);
    //gyro.reset();
    //gyro.resetDisplacement();
    
    kinematics = Control.drivetrain.DRIVE_KINEMATICS;

    poseEstimator = new MecanumDrivePoseEstimator(
      kinematics,
      gyro.getRotation2d(),
      getWheelPositions(),
      new Pose2d(),
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
    );

    /*driveController = new HolonomicDriveController(
      new PIDController(18.5, 0, 2.5), 
      new PIDController(25.0, 0, 2.5),
      new ProfiledPIDController(Control.drivetrain.rotP, Control.drivetrain.rotI, Control.drivetrain.rotD,
          new Constraints(Control.drivetrain.kMaxAngularVelRad, Control.drivetrain.kMaxAngularAccel))); */

    limelight = Control.LIMELIGHT_NAME;

    try{
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
    autoBuilderThing();

    field = new Field2d();
  }

  public void stop(){
    frontLeftSpeed = 0;
    frontRightSpeed = 0;
    rearLeftSpeed = 0;
    rearRightSpeed = 0;
  }

  public void drive(double xSpeed, double ySpeed, double zRotation, boolean fieldRelative){
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, zRotation);
    MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(
      fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, gyro.getRotation2d())
                    : chassisSpeeds
    );
    wheelSpeeds.desaturate(Control.drivetrain.kMaxVelMPS);

    frontLeftSpeed = Util.metersPerSecondToRPM(wheelSpeeds.frontLeftMetersPerSecond, wheelDiameter);
    frontRightSpeed = Util.metersPerSecondToRPM(wheelSpeeds.frontRightMetersPerSecond, wheelDiameter);
    rearLeftSpeed = Util.metersPerSecondToRPM(wheelSpeeds.rearLeftMetersPerSecond, wheelDiameter);
    rearRightSpeed = Util.metersPerSecondToRPM(wheelSpeeds.rearRightMetersPerSecond, wheelDiameter);

    if (Math.abs(xSpeed) < 2.8 && Math.abs(ySpeed) > 3.5){
      frontRightSpeed = - frontLeftSpeed;
      rearLeftSpeed = - rearRightSpeed;
    }

    if (Math.abs(ySpeed) < 2.1 && Math.abs(xSpeed) > 3.5){
      frontRightSpeed = frontLeftSpeed;
      rearLeftSpeed = rearRightSpeed;
    }
  }

  public void drive(ChassisSpeeds chassisSpeeds){
    MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
    wheelSpeeds.desaturate(Control.drivetrain.kMaxVelMPS);

    frontLeftSpeed = Util.metersPerSecondToRPM(wheelSpeeds.frontLeftMetersPerSecond, wheelDiameter);
    frontRightSpeed = Util.metersPerSecondToRPM(wheelSpeeds.frontRightMetersPerSecond, wheelDiameter);
    rearLeftSpeed = Util.metersPerSecondToRPM(wheelSpeeds.rearLeftMetersPerSecond, wheelDiameter);
    rearRightSpeed = Util.metersPerSecondToRPM(wheelSpeeds.rearRightMetersPerSecond, wheelDiameter);
  }

  public void lockDrive(double xSpeed, double ySpeed, Pose2d point){
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, 
      FieldConstraints.Reef.centerFaces[1].minus(getPose2d()).getTranslation().rotateBy(getPose2d().getRotation()).getAngle().getRadians()*0.5);
    MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(
      ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, gyro.getRotation2d())
    );
    wheelSpeeds.desaturate(Control.drivetrain.kMaxVelMPS);

    frontLeftSpeed = Util.metersPerSecondToRPM(wheelSpeeds.frontLeftMetersPerSecond, wheelDiameter);
    frontRightSpeed = Util.metersPerSecondToRPM(wheelSpeeds.frontRightMetersPerSecond, wheelDiameter);
    rearLeftSpeed = Util.metersPerSecondToRPM(wheelSpeeds.rearLeftMetersPerSecond, wheelDiameter);
    rearRightSpeed = Util.metersPerSecondToRPM(wheelSpeeds.rearRightMetersPerSecond, wheelDiameter);

    if (Math.abs(xSpeed) < 2.8 && Math.abs(ySpeed) > 3.5){
      frontRightSpeed = - frontLeftSpeed;
      rearLeftSpeed = - rearRightSpeed;
    }

    if (Math.abs(ySpeed) < 2.1 && Math.abs(xSpeed) > 3.5){
      frontRightSpeed = frontLeftSpeed;
      rearLeftSpeed = rearRightSpeed;
    }
  }

  /*public void driveToPose(List<Waypoint> waypoints){
    PathConstraints constraints = new PathConstraints(
      Control.drivetrain.kMaxVelMeters, rearLeftSpeed, frontRightSpeed, frontLeftSpeed);
  }*/

  private MecanumDriveWheelPositions getWheelPositions(){
    return new MecanumDriveWheelPositions(
      Util.revolutionsToMeters(frontLeftEnc .getPosition(), wheelDiameter) / Control.drivetrain.GEAR_RATIO,
      Util.revolutionsToMeters(frontRightEnc.getPosition(), wheelDiameter) / Control.drivetrain.GEAR_RATIO,
      Util.revolutionsToMeters(rearLeftEnc  .getPosition(), wheelDiameter) / Control.drivetrain.GEAR_RATIO,
      Util.revolutionsToMeters(rearRightEnc .getPosition(), wheelDiameter) / Control.drivetrain.GEAR_RATIO);
  }

  private ChassisSpeeds getChassisSpeeds(){
    return kinematics.toChassisSpeeds(
      new MecanumDriveWheelSpeeds(
        Util.RPMToMetersPerSecond(frontLeftEnc .getVelocity(), wheelDiameter) / Control.drivetrain.GEAR_RATIO, 
        Util.RPMToMetersPerSecond(frontRightEnc.getVelocity(), wheelDiameter) / Control.drivetrain.GEAR_RATIO, 
        Util.RPMToMetersPerSecond(rearLeftEnc  .getVelocity(), wheelDiameter) / Control.drivetrain.GEAR_RATIO, 
        Util.RPMToMetersPerSecond(rearRightEnc .getVelocity(), wheelDiameter) / Control.drivetrain.GEAR_RATIO));
  }

  public double getHeadingDeg(){
    return Units.radiansToDegrees(MathUtil.angleModulus(gyro.getRotation2d().getRadians()));
  }
  public double getHeadingRad(){
    return MathUtil.angleModulus(gyro.getRotation2d().getRadians());
  }

  public Pose2d getPose2d(){
    return poseEstimator.getEstimatedPosition();
  }

  public PathPlannerPath generatePath(List<Pose2d> goal, Rotation2d endState){
    List<Pose2d> pathPoses = new Stack<Pose2d>();
    pathPoses.add(this.getPose2d());
    for (int i = 0; i < goal.size(); i++){
      pathPoses.add(goal.get(i));
    }
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      pathPoses
    );

    return new PathPlannerPath(
      waypoints,
      Control.drivetrain.pathConstraints, 
      null,
      new GoalEndState(0.0, endState)
    );
  }

  public Command followPathCommand(PathPlannerPath path) {
    try {
      return new FollowPathCommand(
        path,
        this::getPose2d, 
        this::getChassisSpeeds, 
        (speeds, feedforwards) -> drive(speeds), 
        Control.drivetrain.PPDriveController,
        robotConfig,
        Util::isRed,
        this);
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }

  public boolean exampleCondition() {
    return false;
  }

  @Override
  public void periodic() {
    LimelightHelpers.SetRobotOrientation(limelight, gyro.getYaw(), 0, 0, 0, 0, 0);
    if (Util.isBlue())
      limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);
    else 
      limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiRed(limelight);
    poseEstimator.update(Rotation2d.fromDegrees(90), getWheelPositions());
    poseEstimator.addVisionMeasurement(limelightMeasurement.pose, 
                                       limelightMeasurement.timestampSeconds, 
                                       VecBuilder.fill(0.7, 0.7, 0.7));

    frontLeftMotor .getClosedLoopController().setReference(frontLeftSpeed,  ControlType.kMAXMotionVelocityControl);
    frontRightMotor.getClosedLoopController().setReference(frontRightSpeed, ControlType.kMAXMotionVelocityControl);
    rearLeftMotor  .getClosedLoopController().setReference(rearLeftSpeed,   ControlType.kMAXMotionVelocityControl);
    rearRightMotor .getClosedLoopController().setReference(rearRightSpeed,  ControlType.kMAXMotionVelocityControl);

    field.setRobotPose(poseEstimator.getEstimatedPosition());

    SmartDashboard.putNumber("flVel", frontLeftEnc.getVelocity());
    SmartDashboard.putNumber("flVel", frontRightEnc.getVelocity());
    SmartDashboard.putNumber("rlVel", rearLeftEnc.getVelocity());
    SmartDashboard.putNumber("rrVel", rearRightEnc.getVelocity());
    SmartDashboard.putNumber("flPosition", Util.revolutionsToMeters(frontLeftEnc.getPosition() / 5.95, wheelDiameter));
    SmartDashboard.putNumber("frPosition", Util.revolutionsToMeters(frontRightEnc.getPosition() / 5.95, wheelDiameter));
    SmartDashboard.putNumber("rlPosition", Util.revolutionsToMeters(rearLeftEnc.getPosition() / 5.95, wheelDiameter));
    SmartDashboard.putNumber("rrPosition", Util.revolutionsToMeters(rearRightEnc.getPosition() / 5.95, wheelDiameter));
    SmartDashboard.putNumber("PoseX", poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("PoseY", poseEstimator.getEstimatedPosition().getY());
    SmartDashboard.putNumber("flSetPoint", frontLeftSpeed);
    SmartDashboard.putNumber("frSetPoint", frontRightSpeed);
    SmartDashboard.putNumber("rlSetPoint", rearLeftSpeed);
    SmartDashboard.putNumber("rrSetPoint", rearRightSpeed);
    SmartDashboard.putNumber("heading", getHeadingDeg());
    SmartDashboard.putData("field", field);
  }

  @Override
  public void simulationPeriodic() {}

  private void configure(){
    config = new SparkMaxConfig();
    /*config.apply(new EncoderConfig().positionConversionFactor(Units.inchesToMeters(6) * Math.PI / 5.95));
    config.apply(new EncoderConfig().velocityConversionFactor(0.1524 / Math.PI * 60));  */
    config.smartCurrentLimit(Control.CURRENT_LIMIT)
          .idleMode(IdleMode.kBrake)
          .inverted(false)
          .closedLoopRampRate(Control.drivetrain.kRampRate)
          .closedLoop.pidf(Control.drivetrain.kP, Control.drivetrain.kI, Control.drivetrain.kD, Control.drivetrain.kflFF)
                     .maxMotion.maxAcceleration(Control.drivetrain.kMaxAccelRPM)
                               .maxVelocity(Control.drivetrain.kMaxVelRPM)
                               .allowedClosedLoopError(Control.drivetrain.kAllowedError)
                               .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal); 
    frontLeftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.apply(new SparkMaxConfig().closedLoop.pidf(Control.drivetrain.kP, Control.drivetrain.kI, Control.drivetrain.kD, Control.drivetrain.krlFF));
    rearLeftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.apply(new SparkMaxConfig().inverted(true));

    config.apply(new SparkMaxConfig().closedLoop.pidf(Control.drivetrain.kP, Control.drivetrain.kI, Control.drivetrain.kD, Control.drivetrain.kfrFF));
    frontRightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.apply(new SparkMaxConfig().closedLoop.pidf(Control.drivetrain.kP, Control.drivetrain.kI, Control.drivetrain.kD, Control.drivetrain.krrFF));
    rearRightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void autoBuilderThing(){
    // Configure AutoBuilder last
    AutoBuilder.configure(
            poseEstimator::getEstimatedPosition, // Robot pose supplier
            poseEstimator::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> drive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            Control.drivetrain.PPDriveController,
            robotConfig, // The robot configuration
            Util::isRed,
            this // Reference to this subsystem to set requirements
    );
  }
}
