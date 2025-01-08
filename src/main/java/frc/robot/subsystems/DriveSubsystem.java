// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.Ports;

public class DriveSubsystem extends SubsystemBase {

  private SparkMax frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor;
  private SparkMaxConfig config;

  public DriveSubsystem() {
    frontLeftMotor = new SparkMax(Ports.drivetrain.FRONT_LEFT, MotorType.kBrushless);
    frontRightMotor = new SparkMax(Ports.drivetrain.FRONT_RIGHT, MotorType.kBrushless);
    rearLeftMotor = new SparkMax(Ports.drivetrain.REAR_LEFT, MotorType.kBrushless);
    rearRightMotor = new SparkMax(Ports.drivetrain.REAR_RIGHT, MotorType.kBrushless);

    config = new SparkMaxConfig();
    config.smartCurrentLimit(10)
          .idleMode(IdleMode.kBrake)
          .closedLoop.maxMotion.maxAcceleration(500)
                               .maxVelocity(4000);
    frontLeftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    frontRightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rearLeftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rearRightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

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
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
