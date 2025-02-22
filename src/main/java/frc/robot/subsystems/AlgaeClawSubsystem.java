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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Control;
import frc.robot.constants.Ports;

public class AlgaeClawSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private SparkMax arm,
                   shootLeader, shootFollower;
  private SparkMaxConfig armConfig, shootConfig;
  private static double armSetPoint, shootSetPoint;
  private static ColorSensorV3 colorSensor;
  private SparkAbsoluteEncoder absEncoder;
  private ArmFeedforward feedforward;
  private ProfiledPIDController pidController;

  public AlgaeClawSubsystem() {
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
    arm.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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


    absEncoder = arm.getAbsoluteEncoder();
    colorSensor = new ColorSensorV3(Ports.manipulator.COLOR_SENSOR);
  }

  private static void armSetSetPoint(double angle){
    armSetPoint = angle;
  }
  private static void shootSetSetPoint(double velRPM){
    shootSetPoint = velRPM;
  }

  public void stow(){
    armSetSetPoint(Control.algaeManipulator.kStowAngle);
    shootSetSetPoint(Control.algaeManipulator.kStopSpeed);
  }
  public void intake(){
    armSetSetPoint(Control.algaeManipulator.kIntakeAngle);
    shootSetSetPoint(Control.algaeManipulator.kIntakeSpeed);
  }
  public void shoot(){
    armSetSetPoint(Control.algaeManipulator.kFixedShootAngle);
    shootSetSetPoint(Control.algaeManipulator.kFixedShootSpeed);; //TODO add algorithm
  }
  public void grab(){
    armSetSetPoint(Control.algaeManipulator.kGrabAngle);
    shootSetSetPoint(Control.algaeManipulator.kGrabSpeed);
  }
  public void grabGround(){
    armSetSetPoint(Control.algaeManipulator.kGroundGrabAngle);
    shootSetSetPoint(Control.algaeManipulator.kGrabSpeed);
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
          /* one-time action goes here */
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
    return absEncoder.getPosition() - armSetPoint < Control.algaeManipulator.kAllowedArmError;
  }
  public boolean atShooterSetPoint() {
    return shootLeader.getEncoder().getVelocity() - shootSetPoint < Control.algaeManipulator.kAllowedShootError;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    arm.getClosedLoopController().setReference(armSetPoint, ControlType.kMAXMotionPositionControl);
    arm.setVoltage(
      feedforward.calculate(armSetPoint, armSetPoint) +
      pidController.calculate(armSetPoint)
    );
    shootLeader.getClosedLoopController().setReference(shootSetPoint, ControlType.kMAXMotionVelocityControl);

    SmartDashboard.putBoolean("algae", hasAlgae());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}