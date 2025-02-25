// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.Control;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private static SparkMax motor;
  private static SparkMaxConfig config;
  private ElevatorFeedforward feedforward;
  private ProfiledPIDController pidController;
  private double setPoint;
  //private ElevatorState elevatorState;
  private ElevatorSubsystem() {
    motor = new SparkMax(Ports.elevator.ELEVATOR, Control.elevator.MOTOR_TYPE);
    config = new SparkMaxConfig();
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
    
    //elevatorState = ElevatorState.NEUTRAL;
    }

    private static ElevatorSubsystem instance;
    public static ElevatorSubsystem getInstance(){
      if (instance == null){
        instance = new ElevatorSubsystem();
      }
      return instance;
    }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command goToL1() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          L1();
        });
  }
  public Command goToL2() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          L2();
        });
  }
  public Command goToL3() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          L3();
        });
  }

  /*public enum ElevatorState {
    NEUTRAL, 
    L1, L2, L3;
  }
  private void stateSwitch(){
    switch (elevatorState){
      case NEUTRAL:
      case L1: 
        setSetPoint(Control.elevator.kL1 + Control.elevator.kOffset);
        break;
      case L2:
      case L3:
        break;
    }
  }*/

  public void neutral(){
    setSetPoint(Control.elevator.kNeutral);
  }
  public void L1(){
    setSetPoint(Control.elevator.kL1);
  }
  public void L2(){
    setSetPoint(Control.elevator.kL2);
  }
  public void L3(){
    setSetPoint(Control.elevator.kL3);
  }

  public void setSetPoint(double setPoint){
    this.setPoint = setPoint;
  }
  public boolean atSetPoint() {
    return Math.abs(motor.getEncoder().getPosition() - setPoint) < Control.elevator.kAllowedError;
  }
  public double getHeight(){
    return /*Control.elevator.kConversionFactor */ motor.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    double feedforwardCalculation = feedforward.calculate(pidController.getSetpoint().velocity);
    double pidCalculation = pidController.calculate(motor.getEncoder().getPosition(), setPoint);
    //L1();
    //stateSwitch();
    //motor.getClosedLoopController().setReference(setPoint, ControlType.kMAXMotionPositionControl);
    motor.setVoltage(
      pidCalculation
    + feedforwardCalculation
    );

    SmartDashboard.putNumber("setpoint", setPoint);
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
