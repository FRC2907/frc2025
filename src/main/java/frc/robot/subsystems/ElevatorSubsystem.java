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
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.constants.Ports;
import frc.robot.util.Util;
import frc.robot.constants.Control;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private static SparkFlex motor, motorFollower;
  private static SparkFlexConfig config;
  private ElevatorFeedforward feedforward;
  private ProfiledPIDController pidController;
  private int index;
  private int indexMax;
  private boolean algaeScoring;
  private ElevatorSubsystem() {
    motor = new SparkFlex(Ports.elevator.LEADER, Control.elevator.MOTOR_TYPE);
    motorFollower = new SparkFlex(Ports.elevator.FOLLOWER, Control.elevator.MOTOR_TYPE);
    config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake)
          .inverted(false)
          .smartCurrentLimit(0, 40)
          .softLimit.forwardSoftLimit(4 * Control.elevator.GEAR_RATIO)
                    .forwardSoftLimitEnabled(true)
                    .reverseSoftLimit(0)
                    .reverseSoftLimitEnabled(false);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorFollower.configure(config.follow(motor, true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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

    index = 0;
    indexMax = 3;

    algaeScoring = false;
  }

  private static ElevatorSubsystem instance;
  public static ElevatorSubsystem getInstance(){
    if (instance == null){
      instance = new ElevatorSubsystem();
    }
    return instance;
  }



  public void stop(){
    motor.stopMotor();
  }

  public void addIndex(){
    if (index < indexMax) index++;
  }
  public void subtractIndex(){
    if (index > 0) index--;
  }
  public void resetIndex(){
    index = 0;
  }
  public void switchScoring(){
    algaeScoring = !algaeScoring;
  }

  public void elevatorRun(){
    if (algaeScoring){
      if (index == 0){ driveMotors(neutral()); }
      if (index == 1){ driveMotors(A1()); }
      if (index == 2){ driveMotors(A2()); }
      if (index == 3){ index = 2; }
    } else {
      if (index == 0){ driveMotors(neutral()); }
      if (index == 1){ driveMotors(L1()); }
      if (index == 2){ driveMotors(L2()); }
      if (index == 3){ driveMotors(L3()); }
    }
  }

  public double ground()      { return Control.elevator.kDownLimit; }
  public double neutral()     { return Control.elevator.kNeutral; }
  public double coralStation(){ return Control.elevator.kCoralStation; }
  public double L1()          { return Control.elevator.kL1; }
  public double L2()          { return Control.elevator.kL2; }
  public double L3()          { return Control.elevator.kL3; }
  public double A1()          { return Control.elevator.kA1; }
  public double A2()          { return Control.elevator.kA2; }
  

  public Command reset(){
    return runOnce(() -> motor.getEncoder().setPosition(0));
  }
  public Command manualDown(){
    return run(() -> motor.set(-0.05));
  }
  public Command manualUp(){
    return run(() -> motor.set(0.05));
  }
  public Command elevatorUp(){
    return runEnd(() -> elevatorRun(), () -> stop())
      .beforeStarting(() -> { pidReset(); addIndex(); }, this)
      .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
      .until(this::atSetpoint);
  }
  public Command elevatorDown(){
    return runEnd(() -> elevatorRun(), () -> stop())
      .beforeStarting(() -> { pidReset(); subtractIndex(); }, this)
      .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
      .until(this::atSetpoint);
  }
  public Command coralStationCommand(){
    return runEnd(() -> driveMotors(coralStation()), () -> stop())
      .beforeStarting(() -> pidReset(), this)
      .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
      .until(this::atSetpoint);
  }
  
  public Command L1Command(){
    return runEnd(() -> driveMotors(L1()), () -> stop())
      .beforeStarting(() -> pidReset())
      .until(this::atSetpoint);
  }
  public Command L2Command(){
    return runEnd(() -> driveMotors(L2()), () -> stop())
      .beforeStarting(() -> pidReset())
      .until(this::atSetpoint);
  }
  public Command L3Command(){
    return runEnd(() -> driveMotors(L3()), () -> stop())
      .beforeStarting(() -> pidReset())
      .until(this::atSetpoint);
  }



  public boolean atSetpoint() {
    return pidController.atGoal();
  }
  public double getHeight(){
    return motor.getEncoder().getPosition() * Control.elevator.kConversionFactor + Control.elevator.kOffset;
  }
  public Command getReefCommand(int level){
    if (level == 1) return L1Command();
    if (level == 2) return L2Command();
    if (level == 3) return L3Command();
    return L3Command();
  }

  private static String SUBSYSTEM_NAME = "Elevator: ";

  public void pidReset(){
    pidController.reset(getHeight());
  }
  public void driveMotors(double setpoint){
    setpoint = Util.clamp(Control.elevator.kDownLimit, setpoint, Control.elevator.kUpLimit);
    double feedforwardCalculation = feedforward.calculate(motor.getEncoder().getPosition() - setpoint * 1);
    double pidCalculation = pidController.calculate(
      getHeight(), setpoint);
    if (Math.abs(pidCalculation) < 0.5) pidCalculation = 0;    
    motor.setVoltage(
      Util.clamp(Control.elevator.kMinVoltage, pidCalculation, Control.elevator.kMaxVoltage)
    );

    SmartDashboard.putNumber(SUBSYSTEM_NAME + "feedforwardCalculation", feedforwardCalculation);
    SmartDashboard.putNumber(SUBSYSTEM_NAME + "pidCalculation", pidCalculation);
    SmartDashboard.putNumber(SUBSYSTEM_NAME + "setpoint", setpoint);
    SmartDashboard.putNumber(SUBSYSTEM_NAME + "pidsetpoint", pidController.getSetpoint().position);
  }

  @Override
  public void periodic() {
    if (this.getCurrentCommand() == null){
      motor.stopMotor();
    }

    SmartDashboard.putNumber(SUBSYSTEM_NAME + "position", motor.getEncoder().getPosition());
    SmartDashboard.putNumber(SUBSYSTEM_NAME + "pidSetpoint", pidController.getSetpoint().position);
    SmartDashboard.putNumber(SUBSYSTEM_NAME + "goal", pidController.getGoal().position);
    SmartDashboard.putNumber(SUBSYSTEM_NAME + "index", index);
    SmartDashboard.putNumber(SUBSYSTEM_NAME + "height", getHeight());
    SmartDashboard.putNumber("L1", Control.elevator.kL1);
    SmartDashboard.putNumber("L2", Control.elevator.kL2);
    SmartDashboard.putNumber("L3", Control.elevator.kL3);
    SmartDashboard.putNumber("A1", Control.elevator.kA1);
    SmartDashboard.putNumber("A2", Control.elevator.kA2);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}