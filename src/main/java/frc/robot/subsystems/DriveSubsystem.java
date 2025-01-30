// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.Waypoint;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util;
import frc.robot.constants.Control;
import frc.robot.constants.Ports;

public class DriveSubsystem extends SubsystemBase {

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
    gyro.reset();
    gyro.resetDisplacement();
    
    kinematics = Control.drivetrain.DRIVE_KINEMATICS;

    poseEstimator = new MecanumDrivePoseEstimator(
      kinematics,
      gyro.getRotation2d(),
      getWheelPositions(),
      new Pose2d(),
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
    );


    try{
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
    autoBuilderThing();
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
    wheelSpeeds.desaturate(Control.drivetrain.kMaxVelMeters);

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
    wheelSpeeds.desaturate(Control.drivetrain.kMaxVelMeters);

    frontLeftSpeed = Util.metersPerSecondToRPM(wheelSpeeds.frontLeftMetersPerSecond, wheelDiameter);
    frontRightSpeed = Util.metersPerSecondToRPM(wheelSpeeds.frontRightMetersPerSecond, wheelDiameter);
    rearLeftSpeed = Util.metersPerSecondToRPM(wheelSpeeds.rearLeftMetersPerSecond, wheelDiameter);
    rearRightSpeed = Util.metersPerSecondToRPM(wheelSpeeds.rearRightMetersPerSecond, wheelDiameter);
  }

  /*public void driveToPose(List<Waypoint> waypoints){
    PathConstraints constraints = new PathConstraints(
      Control.drivetrain.kMaxVelMeters, rearLeftSpeed, frontRightSpeed, frontLeftSpeed);
  }*/

  private MecanumDriveWheelPositions getWheelPositions(){
    return new MecanumDriveWheelPositions(
      Util.revolutionsToMeters(frontLeftEnc .getPosition(), wheelDiameter) / 5.95,
      Util.revolutionsToMeters(frontRightEnc.getPosition(), wheelDiameter) / 5.95,
      Util.revolutionsToMeters(rearLeftEnc  .getPosition(), wheelDiameter) / 5.95,
      Util.revolutionsToMeters(rearRightEnc .getPosition(), wheelDiameter) / 5.95);
  }

  private ChassisSpeeds getChassisSpeeds(){
    return kinematics.toChassisSpeeds(
      new MecanumDriveWheelSpeeds(
        Util.RPMToMetersPerSecond(frontLeftEnc .getVelocity(), wheelDiameter), 
        Util.RPMToMetersPerSecond(frontRightEnc.getVelocity(), wheelDiameter), 
        Util.RPMToMetersPerSecond(rearLeftEnc  .getVelocity(), wheelDiameter), 
        Util.RPMToMetersPerSecond(rearRightEnc .getVelocity(), wheelDiameter)));
  }

  public double getHeading(){
    double heading = gyro.getRotation2d().getDegrees();
    return heading < 0 ? heading % 360 + 360 : heading % 360;
  }

  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public boolean exampleCondition() {
    return false;
  }

  @Override
  public void periodic() {
    poseEstimator.update(new Rotation2d(0), getWheelPositions());

    frontLeftMotor .getClosedLoopController().setReference(frontLeftSpeed,  ControlType.kMAXMotionVelocityControl);
    frontRightMotor.getClosedLoopController().setReference(frontRightSpeed, ControlType.kMAXMotionVelocityControl);
    rearLeftMotor  .getClosedLoopController().setReference(rearLeftSpeed,   ControlType.kMAXMotionVelocityControl);
    rearRightMotor .getClosedLoopController().setReference(rearRightSpeed,  ControlType.kMAXMotionVelocityControl);

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
    SmartDashboard.putNumber("heading", gyro.getAngle());
  }

  @Override
  public void simulationPeriodic() {
    
  }

  private void configure(){
    config = new SparkMaxConfig();
    /*config.apply(new EncoderConfig().positionConversionFactor(Units.inchesToMeters(6) * Math.PI / 5.95));
    config.apply(new EncoderConfig().velocityConversionFactor(0.1524 / Math.PI * 60));  */
    config.smartCurrentLimit(Control.kCurrentLimit)
          .idleMode(IdleMode.kBrake)
          .inverted(false)
          .closedLoopRampRate(Control.drivetrain.kRampRate)
          .closedLoop.pidf(Control.drivetrain.kP, Control.drivetrain.kI, Control.drivetrain.kD, Control.drivetrain.kflFF)
                     .maxMotion.maxAcceleration(Control.drivetrain.kMaxAcceleration)
                               .maxVelocity(Control.drivetrain.kMaxVelRPM)
                               .allowedClosedLoopError(Control.drivetrain.kAllowedError);
                               //.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal); 
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
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(35.0, 2.0, 2.5), // Translation PID constants
                    new PIDConstants(2.5, 0.0, 0.0) // Rotation PID constants
            ),
            robotConfig, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }
}
