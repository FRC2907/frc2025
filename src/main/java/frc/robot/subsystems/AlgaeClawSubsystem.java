// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkAbsoluteEncoder;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Control;
import frc.robot.constants.FieldElements;
import frc.robot.constants.Ports;
import frc.robot.util.Util;

public class AlgaeClawSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private SparkMax arm,
                   shootLeader, shootFollower;
  private SparkMaxConfig armConfig, shootConfig;
  private static double armSetpoint, shootSetpoint;
  private static ColorSensorV3 colorSensor;
  private SparkAbsoluteEncoder absEncoder;
  private ArmFeedforward feedforward;
  private ProfiledPIDController pidController;

  private AlgaeClawSubsystem() {
    arm = new SparkMax(Ports.manipulator.ALGAE_ARM, Control.algaeManipulator.MOTOR_TYPE);
    armConfig = new SparkMaxConfig();
    armConfig.idleMode(IdleMode.kBrake)
             .inverted(false)
             .smartCurrentLimit(0, 40)
             .softLimit.forwardSoftLimit(180)
                       .forwardSoftLimitEnabled(true)
                       .reverseSoftLimit(0)
                       .reverseSoftLimitEnabled(true);
    armConfig.closedLoop.maxMotion.maxAcceleration(2)
                                  .maxVelocity(2)
                                  .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
    armConfig.encoder.positionConversionFactor(Control.algaeManipulator.kArmConversionFactor);
    arm.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    arm.getEncoder().setPosition(0);

    shootLeader =   new SparkMax(Ports.manipulator.ALGAE_SHOOT_LEADER,   Control.algaeManipulator.MOTOR_TYPE);
    shootFollower = new SparkMax(Ports.manipulator.ALGAE_SHOOT_FOLLOWER, Control.algaeManipulator.MOTOR_TYPE);
    shootConfig = new SparkMaxConfig();
    shootConfig.idleMode(IdleMode.kBrake)
               .inverted(false)
               .smartCurrentLimit(0, 40)
               .closedLoop.pidf(0, 0, 0, 0)
                          .maxMotion.allowedClosedLoopError(30)
                                    .maxVelocity(5000)
                                    .maxAcceleration(2500);
    shootLeader.configure(shootConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shootConfig.apply(new SparkMaxConfig().follow(shootLeader, true));
    shootFollower.configure(shootConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    

    feedforward = new ArmFeedforward(Control.algaeManipulator.kS,
                                     Control.algaeManipulator.kG, 
                                     Control.algaeManipulator.kV, 
                                     Control.algaeManipulator.kA);
    pidController = new ProfiledPIDController(Control.algaeManipulator.kP,
                                              Control.algaeManipulator.kI, 
                                              Control.algaeManipulator.kD, 
                                              Control.algaeManipulator.kConstraints);


    //absEncoder = arm.getAbsoluteEncoder();
    colorSensor = new ColorSensorV3(Ports.manipulator.COLOR_SENSOR);
  }

  private static AlgaeClawSubsystem instance;
  public static AlgaeClawSubsystem getInstance(){
    if (instance == null){
      instance = new AlgaeClawSubsystem();
    }
    return instance;
  }



  private static void armSetSetpoint(double angle){
    armSetpoint = angle;
  }
  private static void shootSetSetpoint(double velRPM){
    shootSetpoint = velRPM;
  }

  public void stow(){
    armSetSetpoint(Control.algaeManipulator.kStowAngle);
    shootSetSetpoint(Control.algaeManipulator.kStopSpeed);
  }
  public void intake(){
    armSetSetpoint(Control.algaeManipulator.kIntakeAngle);
    shootSetSetpoint(Control.algaeManipulator.kIntakeSpeed);
  }
  public void fixedShoot(){
    armSetSetpoint(Control.algaeManipulator.kFixedShootAngle);
    shootSetSetpoint(Control.algaeManipulator.kFixedShootSpeed);; //TODO add algorithm
  }
  public void grab(){
    armSetSetpoint(Control.algaeManipulator.kGrabAngle);
    shootSetSetpoint(Control.algaeManipulator.kGrabSpeed);
  }
  public void grabGround(){
    armSetSetpoint(Control.algaeManipulator.kGroundGrabAngle);
    shootSetSetpoint(Control.algaeManipulator.kGrabSpeed);
  }

  private double calculateSpeed(Pose2d point, double height){
    return Math.abs(point.getX()
     - (Util.isBlue() ? 0 : FieldElements.fieldLength)) * Control.algaeManipulator.kDistanceA +
     /*ElevatorSubsystem.getInstance().getHeight() */ height * Control.algaeManipulator.kHeightA
     + Control.algaeManipulator.kShootB;
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          armSetSetpoint(100.0);
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean hasAlgae() {
    return colorSensor.getProximity() > Control.algaeManipulator.kProximityBand;
  }
  public boolean atArmSetPoint() {
    return absEncoder.getPosition() - armSetpoint < Control.algaeManipulator.kAllowedArmError;
  }
  public boolean atShooterSetPoint() {
    return shootLeader.getEncoder().getVelocity() - shootSetpoint < Control.algaeManipulator.kAllowedShootError;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double pidCalculation = pidController.calculate(arm.getEncoder().getPosition(), armSetpoint);
    double feedforwardCalculation = feedforward.calculate(pidController.getSetpoint().velocity, pidController.getConstraints().maxAcceleration);
    //arm.getClosedLoopController().setReference(armSetPoint, ControlType.kMAXMotionPositionControl);
    arm.setVoltage(
      //feedforwardCalculation + 
      pidCalculation
    );
    shootLeader.getClosedLoopController().setReference(shootSetpoint, ControlType.kMAXMotionVelocityControl);

    SmartDashboard.putBoolean("algae", hasAlgae());
    SmartDashboard.putNumber("setpoint", armSetpoint);
    SmartDashboard.putNumber("feedforwardCalculation", feedforwardCalculation);
    SmartDashboard.putNumber("pidCalculation", pidCalculation);
    SmartDashboard.putNumber("position", arm.getEncoder().getPosition());
    SmartDashboard.putNumber("pidSetpoint", pidController.getSetpoint().position);
    SmartDashboard.putNumber("goal", pidController.getGoal().position);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    SmartDashboard.putNumber("shoot", calculateSpeed(new Pose2d(3, 0, Rotation2d.kZero), 1));
  }
}