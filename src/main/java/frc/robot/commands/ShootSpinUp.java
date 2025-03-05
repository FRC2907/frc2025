// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.AlgaeClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ShootSpinUp extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final AlgaeClawSubsystem algaeSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShootSpinUp(AlgaeClawSubsystem algaeSubsystem, ElevatorSubsystem elevatorSubsystem) {
    this.algaeSubsystem = algaeSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(algaeSubsystem, elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    algaeSubsystem.grab();
    elevatorSubsystem.L1();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeSubsystem.stow();
    elevatorSubsystem.neutral();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return algaeSubsystem.hasAlgae();
  }
}
