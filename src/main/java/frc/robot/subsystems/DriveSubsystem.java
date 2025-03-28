// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Stack;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindThenFollowPath;
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
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Control;
import frc.robot.constants.Ports;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.Util;

public class DriveSubsystem extends SubsystemBase {

  private final Field2d field;

  private double frontLeftSpeed, frontRightSpeed, rearLeftSpeed, rearRightSpeed;
  private double wheelDiameter;
  private SparkMax frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor;
  private SparkMaxConfig config;
  private RelativeEncoder frontLeftEnc, frontRightEnc, rearLeftEnc, rearRightEnc;
  private AHRS gyro;
  private boolean gyroFlip;
  private MecanumDrivePoseEstimator poseEstimator;
  private MecanumDriveKinematics kinematics;
  private RobotConfig robotConfig;
  private String limelight;
  private LimelightHelpers.PoseEstimate limelightMeasurement;
  private List<PathPlannerPath> reefPaths;

  private DriveSubsystem() {
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

    gyro = new AHRS(NavXComType.kMXP_SPI);
    gyro.reset();
    gyro.resetDisplacement();
    gyroFlip = false;
    
    kinematics = Control.drivetrain.DRIVE_KINEMATICS;

    poseEstimator = new MecanumDrivePoseEstimator(
      kinematics,
      getRotation2d(),
      getWheelPositions(),
      new Pose2d(),
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
    );
    //poseEstimator.resetPose(new Pose2d(2, 2, Rotation2d.kZero));

    limelight = Control.LIMELIGHT_NAME;

    field = new Field2d();

    reefPaths = new ArrayList<>(12);
    for (int i = 0; i < 12; i++) {
      try {
        reefPaths.add(PathPlannerPath.fromPathFile(i / 2 + 1 + "-" + leftRight(i)));
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
    
    pathPlannerConfigure();
  }

  private static DriveSubsystem instance;
  public static DriveSubsystem getInstance(){
    if (instance == null){
      instance = new DriveSubsystem();
    }
    return instance;
  }


  public void pidTune(double speed){
    frontLeftSpeed =  speed;
    frontRightSpeed = speed;
    rearLeftSpeed =   speed;
    rearRightSpeed =  speed;
  }
  public void stop(){
    frontLeftSpeed = 0;
    frontRightSpeed = 0;
    rearLeftSpeed = 0;
    rearRightSpeed = 0;

    driveMotors(frontLeftSpeed, frontRightSpeed, rearLeftSpeed, rearRightSpeed);
  }
  public void resetGyro(){
    gyro.reset();
  }
  public void flipGyro(){
    if (gyroFlip == false)
      gyroFlip = true;
    else if (gyroFlip == true)
      gyroFlip = false;
  }
  public void drive(double xSpeed, double ySpeed, double zRotation, boolean fieldRelative){
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, zRotation);
    MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(
      fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getRotation2d()) //DO NOT USE
                    : chassisSpeeds
    );
    wheelSpeeds.desaturate(Control.drivetrain.kMaxVelMPS);

    double frontLeftSpeed = Util.metersPerSecondToRPM(wheelSpeeds.frontLeftMetersPerSecond, wheelDiameter);
    double frontRightSpeed = Util.metersPerSecondToRPM(wheelSpeeds.frontRightMetersPerSecond, wheelDiameter);
    double rearLeftSpeed = Util.metersPerSecondToRPM(wheelSpeeds.rearLeftMetersPerSecond, wheelDiameter);
    double rearRightSpeed = Util.metersPerSecondToRPM(wheelSpeeds.rearRightMetersPerSecond, wheelDiameter);
    SmartDashboard.putNumber("flSpeed", frontLeftSpeed);
    SmartDashboard.putNumber("frSpeed", frontRightSpeed);
    SmartDashboard.putNumber("rlSpeed", rearLeftSpeed);
    SmartDashboard.putNumber("rrSpeed", rearRightSpeed);

    if (Math.abs(xSpeed) < 2.8 && Math.abs(ySpeed) > 3.5){
      frontRightSpeed = - frontLeftSpeed;
      rearLeftSpeed = - rearRightSpeed;
    }

    if (Math.abs(ySpeed) < 2.1 && Math.abs(xSpeed) > 3.5){
      frontRightSpeed = frontLeftSpeed;
      rearLeftSpeed = rearRightSpeed;
    }

    driveMotors(frontLeftSpeed, frontRightSpeed, rearLeftSpeed, rearRightSpeed);
  }

  public void drive(ChassisSpeeds chassisSpeeds){
    MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
    wheelSpeeds.desaturate(Control.drivetrain.kMaxVelMPS);

    double frontLeftSpeed = Util.metersPerSecondToRPM(wheelSpeeds.frontLeftMetersPerSecond, wheelDiameter);
    double frontRightSpeed = Util.metersPerSecondToRPM(wheelSpeeds.frontRightMetersPerSecond, wheelDiameter);
    double rearLeftSpeed = Util.metersPerSecondToRPM(wheelSpeeds.rearLeftMetersPerSecond, wheelDiameter);
    double rearRightSpeed = Util.metersPerSecondToRPM(wheelSpeeds.rearRightMetersPerSecond, wheelDiameter);

    driveMotors(frontLeftSpeed, frontRightSpeed, rearLeftSpeed, rearRightSpeed);
  }

  public void lockDrive(double xSpeed, double ySpeed, Pose2d point, boolean limelightLock){
    double atan = (Math.atan((point.getY() - getPose2d().getY()) / point.getX() - getPose2d().getX()) - getHeadingRad()) * 20;
    double zRotation = limelightLock 
    ?   Units.degreesToRadians( - LimelightHelpers.getTX(limelight)) * Control.drivetrain.kHeadingP
      + Units.degreesToRadians(gyro.getRate()) * Control.drivetrain.kHeadingD
    :   atan
      + Units.degreesToRadians(gyro.getRate()) * Control.drivetrain.kHeadingD;
    drive(xSpeed, ySpeed, zRotation, false);

    SmartDashboard.putNumber("pointX", point.getX());
    SmartDashboard.putNumber("pointY", point.getY());
    SmartDashboard.putNumber("atan", atan);
    SmartDashboard.putNumber("arate", Units.degreesToRadians(gyro.getRate()) * Control.drivetrain.kHeadingD);
  }


  private MecanumDriveWheelPositions getWheelPositions(){
    return new MecanumDriveWheelPositions(
      Util.revolutionsToMeters(frontLeftEnc .getPosition(), wheelDiameter) / Control.drivetrain.GEAR_RATIO,
      Util.revolutionsToMeters(frontRightEnc.getPosition(), wheelDiameter) / Control.drivetrain.GEAR_RATIO,
      Util.revolutionsToMeters(rearLeftEnc  .getPosition(), wheelDiameter) / Control.drivetrain.GEAR_RATIO,
      Util.revolutionsToMeters(rearRightEnc .getPosition(), wheelDiameter) / Control.drivetrain.GEAR_RATIO);
  }
  public ChassisSpeeds getChassisSpeeds(){
    return kinematics.toChassisSpeeds(
      new MecanumDriveWheelSpeeds(
        Util.RPMToMetersPerSecond(frontLeftEnc .getVelocity(), wheelDiameter) / Control.drivetrain.GEAR_RATIO, 
        Util.RPMToMetersPerSecond(frontRightEnc.getVelocity(), wheelDiameter) / Control.drivetrain.GEAR_RATIO, 
        Util.RPMToMetersPerSecond(rearLeftEnc  .getVelocity(), wheelDiameter) / Control.drivetrain.GEAR_RATIO, 
        Util.RPMToMetersPerSecond(rearRightEnc .getVelocity(), wheelDiameter) / Control.drivetrain.GEAR_RATIO));
  }

  public Rotation2d getRotation2d(){
    return gyroFlip ? gyro.getRotation2d().plus(Rotation2d.fromDegrees(180))
                    : gyro.getRotation2d();
  }
  public double getHeadingDeg(){
    return Units.radiansToDegrees(MathUtil.angleModulus(getRotation2d().getRadians()));
  }
  public double getHeadingRad(){
    return MathUtil.angleModulus(getRotation2d().getRadians());
  }

  public Pose2d getPose2d(){
    return poseEstimator.getEstimatedPosition();
  }

  public PathPlannerPath generatePath(List<Pose2d> goal, Rotation2d endState){
    List<Pose2d> pathPoses = new Stack<Pose2d>();
    //pathPoses.add(this.getPose2d());
    for (int i = 0; i < goal.size(); i++){
      pathPoses.add(goal.get(i));
    }
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      pathPoses
    );

    return new PathPlannerPath(
      waypoints,
      Control.drivetrain.kPathConstraints, 
      null,
      new GoalEndState(0.0, endState)
    );
  }

  private String leftRight(int index){
    return index % 2 == 0 ? "L" : "R";
  }

  public PathPlannerPath getNearestPath() { return getNearestPath(getPose2d()); }
  public PathPlannerPath getNearestPath(Pose2d here){
    return reefPaths.stream()
      .map(
        path -> new Pair<>(path
          , path
              .getPathPoses()
              .get(0)
              .minus(here)
              .getTranslation()
              .getNorm()
        )
      ).reduce(
        (a,b) -> a.getSecond().doubleValue() < b.getSecond().doubleValue() ? a : b
      ).get().getFirst();
  }
  public PathPlannerPath getPathLeft() { return getPathLeft(getNearestPath()); }
  public PathPlannerPath getPathRight() { return getPathRight(getNearestPath()); }
  public PathPlannerPath getPathLeft(PathPlannerPath current) {
    return reefPaths.get(pathIndexWrap(reefPaths.indexOf(current) + 1));
  }
  public PathPlannerPath getPathRight(PathPlannerPath current) {
    return reefPaths.get(pathIndexWrap(reefPaths.indexOf(current) - 1));
  }
  private int pathIndexWrap(int index){
    if (index > 11)
      return 0;
    if (index < 0)
      return 11;
    return index;
  }

  public Command followPathCommand(PathPlannerPath path) {
    return new PathfindThenFollowPath(
      path,
      Control.drivetrain.kPathConstraints, 
      this::getPose2d, 
      this::getChassisSpeeds, 
      (speeds, feedforwards) -> drive(speeds), 
      Control.drivetrain.PPDriveController, 
      robotConfig, 
      Util::isRed, 
      this);
  }

  private void driveMotors(double frontLeftSpeed, double frontRightSpeed, double rearLeftSpeed, double rearRightSpeed){
    SmartDashboard.putNumber(SUBSYSTEM_NAME + "flSetPoint", frontLeftSpeed);
    SmartDashboard.putNumber(SUBSYSTEM_NAME + "frSetPoint", frontRightSpeed);
    SmartDashboard.putNumber(SUBSYSTEM_NAME + "rlSetPoint", rearLeftSpeed);
    SmartDashboard.putNumber(SUBSYSTEM_NAME + "rrSetPoint", rearRightSpeed);

    frontLeftSpeed *= Control.drivetrain.GEAR_RATIO;
    frontRightSpeed *= Control.drivetrain.GEAR_RATIO;
    rearLeftSpeed *= Control.drivetrain.GEAR_RATIO;
    rearRightSpeed *= Control.drivetrain.GEAR_RATIO;
    frontLeftMotor .getClosedLoopController().setReference(frontLeftSpeed,  ControlType.kMAXMotionVelocityControl);
    frontRightMotor.getClosedLoopController().setReference(frontRightSpeed, ControlType.kMAXMotionVelocityControl);
    rearLeftMotor  .getClosedLoopController().setReference(rearLeftSpeed,   ControlType.kMAXMotionVelocityControl);
    rearRightMotor .getClosedLoopController().setReference(rearRightSpeed,  ControlType.kMAXMotionVelocityControl);
  }

  private static String SUBSYSTEM_NAME = "Drive: ";


  @Override
  public void periodic() {
    //LimelightHelpers.SetRobotOrientation(limelight, gyro.getYaw(), 0, 0, 0, 0, 0);
    if (Util.isBlue())
      limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);
    else 
      limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiRed(limelight);
    poseEstimator.update(getRotation2d(), getWheelPositions());
    if (limelightMeasurement != null && limelightMeasurement.tagCount > 0){
      poseEstimator.addVisionMeasurement(limelightMeasurement.pose, 
                                         limelightMeasurement.timestampSeconds, 
                                         VecBuilder.fill(0.7, 0.7, 0.7));
    }


    field.setRobotPose(poseEstimator.getEstimatedPosition());

    SmartDashboard.putNumber(SUBSYSTEM_NAME + "flVel", frontLeftEnc.getVelocity());
    SmartDashboard.putNumber(SUBSYSTEM_NAME + "frVel", frontRightEnc.getVelocity());
    SmartDashboard.putNumber(SUBSYSTEM_NAME + "rlVel", rearLeftEnc.getVelocity());
    SmartDashboard.putNumber(SUBSYSTEM_NAME + "rrVel", rearRightEnc.getVelocity());
    SmartDashboard.putNumber(SUBSYSTEM_NAME + "flPosition", Util.revolutionsToMeters(frontLeftEnc.getPosition() / 7.31, wheelDiameter));
    SmartDashboard.putNumber(SUBSYSTEM_NAME + "frPosition", Util.revolutionsToMeters(frontRightEnc.getPosition() / 7.31, wheelDiameter));
    SmartDashboard.putNumber(SUBSYSTEM_NAME + "rlPosition", Util.revolutionsToMeters(rearLeftEnc.getPosition() / 7.31, wheelDiameter));
    SmartDashboard.putNumber(SUBSYSTEM_NAME + "rrPosition", Util.revolutionsToMeters(rearRightEnc.getPosition() / 7.31, wheelDiameter));
    SmartDashboard.putNumber(SUBSYSTEM_NAME + "PoseX", poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber(SUBSYSTEM_NAME + "PoseY", poseEstimator.getEstimatedPosition().getY());

    SmartDashboard.putNumber(SUBSYSTEM_NAME + "heading", getHeadingDeg());
    SmartDashboard.putData("field", field);

    //SmartDashboard.putNumber("index", currentPathIndex);
    //SmartDashboard.putNumber("otherIndex", closestPoseIndex);
  }

  @Override
  public void simulationPeriodic() {}

  private void configure(){
    config = new SparkMaxConfig();
    /*config.apply(new EncoderConfig().positionConversionFactor(Control.drivetrain.kPositionConversionFactor)
                                    .velocityConversionFactor(Control.drivetrain.kPositionConversionFactor));*/
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

  private void pathPlannerConfigure(){
    try{
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
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