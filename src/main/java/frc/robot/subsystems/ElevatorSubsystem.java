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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.util.Util;
import frc.robot.constants.Control;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private static SparkFlex motor, motorFollower;
  private static SparkFlexConfig config;
  private ElevatorFeedforward feedforward;
  private ProfiledPIDController pidController;
  private double setpoint;
  private int index;
  private int indexMax;
  private ElevatorSubsystem() {
    motor = new SparkFlex(Ports.elevator.LEADER, Control.elevator.MOTOR_TYPE);
    motorFollower = new SparkFlex(Ports.elevator.FOLLOWER, Control.elevator.MOTOR_TYPE);
    config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake)
          .inverted(true)
          .smartCurrentLimit(0, 40)
          .softLimit.forwardSoftLimit(14.5)
                    .forwardSoftLimitEnabled(true)
                    .reverseSoftLimit(0)
                    .reverseSoftLimitEnabled(true);
    config.encoder.positionConversionFactor(Control.elevator.kConversionFactor);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.apply(new SparkFlexConfig().follow(motor, true));
    motorFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    motor.getEncoder().setPosition(0);
    motorFollower.getEncoder().setPosition(0);

    feedforward = new ElevatorFeedforward(Control.elevator.kS,
                                          Control.elevator.kG, 
                                          Control.elevator.kV, 
                                          Control.elevator.kA);
    pidController = new ProfiledPIDController(Control.elevator.kP,
                                              Control.elevator.kI, 
                                              Control.elevator.kD, 
                                              Control.elevator.kConstraints);
    
    setpoint = 0;
    index = 0;
    indexMax = 3;
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
          if (index < indexMax){ 
          index++; 
        }
          elevatorSwitch();
          System.out.println("working");
        });
  }
  public Command down(){
    return runOnce(
        () -> {
          if (index > 0){ index--; }
          elevatorSwitch();
        });
  }
  public Command manualUp(){
    return run(
      () -> {
        setSetpoint(setpoint + Control.elevator.kManualControlFactor);
        driveMotors(setpoint);
      });
  }
  public Command manualDown(){
    return run(
      () -> {
        setSetpoint(setpoint - Control.elevator.kManualControlFactor);
        driveMotors(setpoint);
      });
  }

  public Command coralStationCommand(){
    return runOnce(
      () -> {
        coralStation();;
        
      });
  }

  public RunCommand testDown(){
    return new RunCommand(() -> motor.set(0.1), this);
  }
  public RunCommand testUp(){
    return new RunCommand(() -> motor.set(-0.1), this);
  }

  private void elevatorSwitch(){
    if (index == 0){ neutral();; driveMotors(setpoint); }
    if (index == 1){ L1();; driveMotors(setpoint); }
    if (index == 2){ L2();; driveMotors(setpoint); }
    if (index == 3){ L3();; driveMotors(setpoint); }
  }

  public void ground()      { setSetpoint(Control.elevator.kDownLimit); }
  public void neutral()     { setSetpoint(Control.elevator.kNeutral); }
  public void processor()   { setSetpoint(Control.elevator.kProcessor); }
  public void coralStation(){ setSetpoint(Control.elevator.kCoralStation); }
  public void L1()          { setSetpoint(Control.elevator.kL1); }
  public void L2()          { setSetpoint(Control.elevator.kL2); }
  public void L3()          { setSetpoint(Control.elevator.kL3); }


  public void setSetpoint(double setpoint){
    this.setpoint = setpoint;
  }
  public boolean atSetpoint() {
    return Math.abs(motor.getEncoder().getPosition() - setpoint) < Control.elevator.kAllowedError;
  }
  public double getHeight(){
    return motor.getEncoder().getPosition();
  }

  private static String subsystem = "Elevator: ";

  private void driveMotors(double setpoint){
    setpoint = Util.clamp(0, setpoint, Units.inchesToMeters(24));
    double feedforwardCalculation = feedforward.calculate(motor.getEncoder().getPosition() - setpoint * 1);
    double pidCalculation = pidController.calculate(
      motor.getEncoder().getPosition(), setpoint);
    if (Math.abs(feedforwardCalculation) < 0.1) feedforwardCalculation = 0;    
    motor.setVoltage(
      pidCalculation
    //+ 
    //feedforwardCalculation
    );

    SmartDashboard.putNumber(subsystem + "feedforwardCalculation", feedforwardCalculation);
    SmartDashboard.putNumber(subsystem + "pidCalculation", pidCalculation);
    SmartDashboard.putNumber(subsystem + "setpoint", setpoint);
  }
  @Override
  public void periodic() {
    //elevatorSwitch();
    /*double feedforwardCalculation = feedforward.calculate(motor.getEncoder().getPosition() - setpoint * 0.1);
    double pidCalculation = pidController.calculate(
      motor.getEncoder().getPosition() * Control.elevator.kConversionFactor, setpoint);
    //motor.getClosedLoopController().setReference(setPoint, ControlType.kMAXMotionPositionControl);
    motor.setVoltage(
      //pidCalculation
    + feedforwardCalculation
    );*/
    if (!(testUp().isScheduled() && testDown().isScheduled())){
      motor.stopMotor();
    }

    SmartDashboard.putNumber(subsystem + "position", motor.getEncoder().getPosition());
    SmartDashboard.putNumber(subsystem + "pidSetpoint", pidController.getSetpoint().position);
    SmartDashboard.putNumber(subsystem + "goal", pidController.getGoal().position);
    SmartDashboard.putNumber(subsystem + "index", index);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public boolean checkJoystickControl(PS5Controller input, boolean up){
    double deadband = Control.elevator.kElevatorDriverDeadband;
    if (Util.checkDriverDeadband(input)){
      if (up ? input.getLeftY() > deadband : input.getLeftY() < - deadband){
        return true;
      }
    }
    return false;
  }
}
