// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.Control;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private static SparkFlex motor;
  private static SparkFlexConfig config;
  private ElevatorFeedforward feedforward;
  private ProfiledPIDController pidController;
  private double setpoint;
  private int index;
  private int indexMax;
  private ElevatorSubsystem() {
    motor = new SparkFlex(Ports.elevator.ELEVATOR, Control.elevator.MOTOR_TYPE);
    config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake)
          .inverted(false)
          .smartCurrentLimit(0, 40)
          .softLimit.forwardSoftLimit(10)
                    .forwardSoftLimitEnabled(false)
                    .reverseSoftLimit(0)
                    .reverseSoftLimitEnabled(true);
    config.encoder.positionConversionFactor(Control.elevator.kConversionFactor);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motor.getEncoder().setPosition(0);

    feedforward = new ElevatorFeedforward(Control.elevator.kS,
                                          Control.elevator.kG, 
                                          Control.elevator.kV, 
                                          Control.elevator.kA);
    pidController = new ProfiledPIDController(Control.elevator.kP,
                                              Control.elevator.kI, 
                                              Control.elevator.kD, 
                                              Control.elevator.kConstraints);
    
    index = 0;
    indexMax = 4;
  }

  private static ElevatorSubsystem instance;
  public static ElevatorSubsystem getInstance(){
    if (instance == null){
      instance = new ElevatorSubsystem();
    }
    return instance;
  }



  public Command up(){
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          if (index < indexMax){ index++; }
        });
  }
  public Command manualUp(){
    return runOnce(
      () -> {
        setSetpoint(setpoint + Control.elevator.kManualControlFactor);
      });
  }
  public Command down(){
    return runOnce(
        () -> {
          if (index > 0){ index--; }
        });
  }
  public Command manualDown(){
    return runOnce(
      () -> {
        setSetpoint(setpoint - Control.elevator.kManualControlFactor);
      });
  }

  private void elevatorSwitch(){
    if (index == 0){ neutral();; }
    if (index == 1){ L1();; }
    if (index == 2){ L2();; }
    if (index == 3){ L3();; }
  }

  public void ground(){  setSetpoint(Control.elevator.kDownLimit); }
  public void neutral(){ setSetpoint(Control.elevator.kNeutral); }
  public void L1(){      setSetpoint(Control.elevator.kL1); }
  public void L2(){      setSetpoint(Control.elevator.kL2); }
  public void L3(){      setSetpoint(Control.elevator.kL3); }


  public void setSetpoint(double setpoint){
    this.setpoint = setpoint;
  }
  public boolean atSetpoint() {
    return Math.abs(motor.getEncoder().getPosition() - setpoint) < Control.elevator.kAllowedError;
  }
  public double getHeight(){
    return motor.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    elevatorSwitch();
    double feedforwardCalculation = feedforward.calculate(pidController.getSetpoint().velocity);
    double pidCalculation = pidController.calculate(motor.getEncoder().getPosition(), setpoint);
    //motor.getClosedLoopController().setReference(setPoint, ControlType.kMAXMotionPositionControl);
    motor.setVoltage(
      pidCalculation
    //+ feedforwardCalculation
    );

    SmartDashboard.putNumber("setpoint", setpoint);
    SmartDashboard.putNumber("feedforwardCalculation", feedforwardCalculation);
    SmartDashboard.putNumber("pidCalculation", pidCalculation);
    SmartDashboard.putNumber("position", motor.getEncoder().getPosition());
    SmartDashboard.putNumber("pidSetpoint", pidController.getSetpoint().position);
    SmartDashboard.putNumber("goal", pidController.getGoal().position);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
