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
    pidController.setTolerance(Control.elevator.kAllowedError);
    
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



  public void stop(){
    motor.stopMotor();
    setpoint = getHeight();
  }

  public void addIndex(){
    if (index < indexMax) index++;
    System.out.println("called");
  }
  public void subtractIndex(){
    if (index > 0) index--;
  }

  public RunCommand testDown(){
    return new RunCommand(() -> motor.set(0.2), this);
  }
  public RunCommand testUp(){
    return new RunCommand(() -> motor.set(-0.2), this);
  }
  public RunCommand moreTest(){
    return new RunCommand(() -> { motor.setVoltage(0.1); System.out.println("working");}, this);
  }

  public void elevatorRun(){
    if (index == 0){ driveMotors(neutral()); }
    if (index == 1){ driveMotors(L1()); }
    if (index == 2){ driveMotors(L2()); }
    if (index == 3){ driveMotors(L3()); }
  }

  public double ground()      { setSetpoint(Control.elevator.kDownLimit);
                                return Control.elevator.kDownLimit; }
  public double neutral()     { setSetpoint(Control.elevator.kNeutral);
                                return Control.elevator.kNeutral; }
  public double processor()   { setSetpoint(Control.elevator.kProcessor);
                                return Control.elevator.kProcessor; }
  public double coralStation(){ setSetpoint(Control.elevator.kCoralStation);
                                return Control.elevator.kCoralStation; }
  public double L1()          { setSetpoint(Control.elevator.kL1);
                                return Control.elevator.kL1; }
  public double L2()          { setSetpoint(Control.elevator.kL2);
                                return Control.elevator.kL2; }
  public double L3()          { setSetpoint(Control.elevator.kL3);
                                return Control.elevator.kL3; }
  public double manualUp()    { setSetpoint(setpoint + Control.elevator.kManualControlFactor);
                                return setpoint; }
  public double manualDown()  { setSetpoint(setpoint - Control.elevator.kManualControlFactor);
                                return setpoint; }



  public void setSetpoint(double setpoint){
    this.setpoint = setpoint;
  }
  public boolean atSetpoint() {
    return pidController.atGoal();
  }
  public double getHeight(){
    return motor.getEncoder().getPosition() * Control.elevator.kConversionFactor;
  }

  private static String SUBSYSTEM_NAME = "Elevator: ";

  public void driveMotors(double setpoint){
    setpoint = Util.clamp(Control.elevator.kDownLimit, setpoint, Units.inchesToMeters(40));
    double feedforwardCalculation = feedforward.calculate(motor.getEncoder().getPosition() - setpoint * 1);
    double pidCalculation = pidController.calculate(
      motor.getEncoder().getPosition() * Control.elevator.kConversionFactor, setpoint);
    if (Math.abs(feedforwardCalculation) < 0.1) feedforwardCalculation = 0;    
    motor.setVoltage(
      Util.clamp(Control.elevator.kMinVoltage, pidCalculation, Control.elevator.kMaxVoltage)
    );

    SmartDashboard.putNumber(SUBSYSTEM_NAME + "feedforwardCalculation", feedforwardCalculation);
    SmartDashboard.putNumber(SUBSYSTEM_NAME + "pidCalculation", pidCalculation);
    SmartDashboard.putNumber(SUBSYSTEM_NAME + "setpoint", setpoint);
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
    if (this.getCurrentCommand() == null){
      motor.stopMotor();
    }

    SmartDashboard.putNumber(SUBSYSTEM_NAME + "position", motor.getEncoder().getPosition());
    SmartDashboard.putNumber(SUBSYSTEM_NAME + "pidSetpoint", pidController.getSetpoint().position);
    SmartDashboard.putNumber(SUBSYSTEM_NAME + "goal", pidController.getGoal().position);
    SmartDashboard.putNumber(SUBSYSTEM_NAME + "index", index);
    SmartDashboard.putNumber("L1", Control.elevator.kL1);
    SmartDashboard.putNumber("L2", Control.elevator.kL2);
    SmartDashboard.putNumber("L3", Control.elevator.kL3);
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