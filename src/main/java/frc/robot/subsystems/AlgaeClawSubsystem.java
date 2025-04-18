// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Control;
import frc.robot.constants.FieldElements;
import frc.robot.constants.Ports;
import frc.robot.util.Util;

public class AlgaeClawSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private static SparkMax arm,
                          shoot, shootFollower;
  private SparkMaxConfig armConfig, shootConfig;
  private static double armSetpoint, shootSetpoint;
  private static ColorSensorV3 colorSensor;
  private RelativeEncoder absEncoder;
  private ArmFeedforward feedforward;
  private ProfiledPIDController pidController;

  private AlgaeClawSubsystem() {
    arm = new SparkMax(Ports.manipulator.ALGAE_ARM, Control.algaeManipulator.MOTOR_TYPE);
    armConfig = new SparkMaxConfig();
    armConfig.idleMode(IdleMode.kBrake)
             .inverted(true)
             .smartCurrentLimit(0, 40)
             .softLimit.forwardSoftLimit(180)
                       .forwardSoftLimitEnabled(true)
                       .reverseSoftLimit(0)
                       .reverseSoftLimitEnabled(true);
    armConfig.closedLoop.maxMotion.maxAcceleration(2)
                                  .maxVelocity(2)
                                  .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
    arm.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    arm.getEncoder().setPosition(0);

    shoot =         new SparkMax(Ports.manipulator.ALGAE_SHOOT_LEADER,   Control.algaeManipulator.MOTOR_TYPE);
    shootFollower = new SparkMax(Ports.manipulator.ALGAE_SHOOT_FOLLOWER, Control.algaeManipulator.MOTOR_TYPE);
    shootConfig = new SparkMaxConfig();
    shootConfig.idleMode(IdleMode.kBrake)
               .inverted(false)
               .smartCurrentLimit(0, 40)
               .closedLoop.pidf(Control.algaeManipulator.kShootP,
                                Control.algaeManipulator.kShootI,
                                Control.algaeManipulator.kShootD,
                                Control.algaeManipulator.kShootFF)
                          .maxMotion.allowedClosedLoopError(100)
                                    .maxVelocity(3000)
                                    .maxAcceleration(2500);
    shoot.configure(shootConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shootFollower.configure(shootConfig.follow(shoot, true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    

    feedforward = new ArmFeedforward(Control.algaeManipulator.kS,
                                     Control.algaeManipulator.kG, 
                                     Control.algaeManipulator.kV, 
                                     Control.algaeManipulator.kA);
    pidController = new ProfiledPIDController(Control.algaeManipulator.kArmP,
                                              Control.algaeManipulator.kArmI, 
                                              Control.algaeManipulator.kArmD, 
                                              Control.algaeManipulator.kConstraints);


    absEncoder = arm.getAlternateEncoder();
    colorSensor = new ColorSensorV3(Ports.manipulator.COLOR_SENSOR);
  }

  private static AlgaeClawSubsystem instance;
  public static AlgaeClawSubsystem getInstance(){
    if (instance == null){
      instance = new AlgaeClawSubsystem();
    }
    return instance;
  }


  public void stop(){
    arm.stopMotor();
    shoot.stopMotor();
    shootFollower.stopMotor();
  }
  public void armStop(){
    arm.stopMotor();
  }
  public void shootStop(){
    shoot.stopMotor();
    shootFollower.stopMotor();
  }




  public void poop(){
    driveShoot(Control.algaeManipulator.kIntakeSpeed);
  }
  public void stow(){
    driveArm(Control.algaeManipulator.kStowAngle);
    driveShoot(Control.algaeManipulator.kStopSpeed);
  }
  public void intakeAngle(){
    driveArm(Control.algaeManipulator.kIntakeAngle);
  }
  public void intake(){
    driveArm(Control.algaeManipulator.kIntakeAngle);
    driveShoot(Control.algaeManipulator.kIntakeSpeed);
  }
  public void processorAngle(){
    driveArm(Control.algaeManipulator.kProcessorAngle);
  }
  public void processor(){
    driveArm(Control.algaeManipulator.kProcessorAngle);
    driveShoot(Control.algaeManipulator.kIntakeSpeed);
  }
  public void fixedShoot(){
    driveArm(Control.algaeManipulator.kFixedShootAngle);
    driveShoot(Control.algaeManipulator.kFixedShootSpeed);;
  }
  public void grab(){
    driveArm(Control.algaeManipulator.kGrabAngle);
    driveShoot(Control.algaeManipulator.kGrabSpeed);
  }
  public void grabGround(){
    driveArm(Control.algaeManipulator.kGroundGrabAngle);
    driveShoot(Control.algaeManipulator.kGrabSpeed);
  }
  public void shootSpinUp(){
    driveArm(Control.algaeManipulator.kStowAngle);
    driveShoot(calculateSpeed(DriveSubsystem.getInstance().getPose2d(),
                              ElevatorSubsystem.getInstance().getHeight()));
  }
  public void shootRelease(){
    driveArm(Control.algaeManipulator.kFixedShootAngle);
    driveShoot(calculateSpeed(DriveSubsystem.getInstance().getPose2d(),
                              ElevatorSubsystem.getInstance().getHeight()));
  }

  private double calculateSpeed(Pose2d point, double height){
    return Math.abs(point.getX()
     - (Util.isBlue() ? 0 : FieldElements.fieldLength)) * Control.algaeManipulator.kDistanceA
     + height * Control.algaeManipulator.kHeightA
     + Control.algaeManipulator.kShootB;
  }

  
  public RunCommand testUp(){
    return new RunCommand(() -> arm.set(0.1), this);
  }
  public RunCommand testDown(){
    return new RunCommand(() -> arm.set(-0.1), this);
  }
  public RunCommand testShoot(){
    return new RunCommand(() -> shoot.set(0.3), this);
  }

  public Command algaePoop(){ return run(() -> poop()); }
  public Command intakeAlgae(){ return run(() -> intakeAngle()).until(this::atArmSetPoint); }
  public Command shootPrep(){ return run(() -> shootSpinUp()); }
  public Command shoot(){ return run(() -> shootRelease()); }


  public boolean hasAlgae() {
    return colorSensor.getProximity() > Control.algaeManipulator.kProximityBand;
  }
  public boolean atArmSetPoint() {
    return pidController.atSetpoint();
  }
  public boolean atShooterSetPoint() {
    return Math.abs(shoot.getEncoder().getVelocity() - shootSetpoint) < Control.algaeManipulator.kAllowedShootError;
  }
  public double getArmPosition(){
    return Units.degreesToRadians(absEncoder.getPosition() * 360);
  }
  public double getSpeed(){
    return shoot.getEncoder().getVelocity();
  }



  public void pidReset(){
    pidController.reset(getArmPosition());
  }
  public void driveArm(double setpoint){
    //setpoint = setpoint * Control.algaeManipulator.kArmConversionFactor; i don't think this is necessary???
    double pidCalculation = pidController.calculate(getArmPosition(), armSetpoint);
    double feedforwardCalculation = feedforward.calculate(
      pidController.getSetpoint().position, pidController.getSetpoint().velocity);
    arm.setVoltage(
      feedforwardCalculation + 
      pidCalculation
    );

    SmartDashboard.putNumber("feedforwardCalculation", feedforwardCalculation);
    SmartDashboard.putNumber("pidCalculation", pidCalculation);
  }
  public void driveShoot(double setpoint){
    shoot.getClosedLoopController().setReference(shootSetpoint, ControlType.kMAXMotionVelocityControl);
  }

  private static String SUBSYSTEM_NAME = "Algae: ";

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (this.getCurrentCommand() == null){
      arm.stopMotor();
      shoot.stopMotor();
      shootFollower.stopMotor();
    }

    SmartDashboard.putBoolean(SUBSYSTEM_NAME + "algae", hasAlgae());
    SmartDashboard.putNumber(SUBSYSTEM_NAME + "setpoint", armSetpoint);
    SmartDashboard.putNumber(SUBSYSTEM_NAME + "position", Units.radiansToDegrees(absEncoder.getPosition()));
    SmartDashboard.putNumber(SUBSYSTEM_NAME + "pidSetpoint", pidController.getSetpoint().position);
    SmartDashboard.putNumber(SUBSYSTEM_NAME + "goal", pidController.getGoal().position);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    SmartDashboard.putNumber("shoot", calculateSpeed(new Pose2d(3, 0, Rotation2d.kZero), 1));
  }
}