// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.Control;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private static SparkMax motor;
  private static SparkMaxConfig config;
  private double setPoint;
  public ElevatorSubsystem() {
    motor = new SparkMax(Ports.elevator.ELEVATOR, Control.elevator.MOTOR_TYPE);
    config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake)
          .inverted(false)
          .smartCurrentLimit(0, 40)
          .softLimit.forwardSoftLimit(30)
                    .forwardSoftLimitEnabled(true)
                    .reverseSoftLimit(0)
                    .reverseSoftLimitEnabled(true);
    config.closedLoop.maxMotion.maxAcceleration(2)
                               .maxVelocity(2)
                               .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
    config.encoder.positionConversionFactor(Control.elevator.kConversionFactor);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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

  public void L1(){
    setSetpoint(Control.elevator.kL1);
  }
  public void setSetpoint(double setPoint){
    this.setPoint = setPoint - Control.elevator.kElevatorOffset;
  }
  public boolean atSetPoint() {
    return Math.abs(motor.getEncoder().getPosition() - setPoint) < Control.elevator.kAllowedError;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    motor.getClosedLoopController().setReference(setPoint, ControlType.kMAXMotionPositionControl);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
