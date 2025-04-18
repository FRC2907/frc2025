// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.grabAlgae;

import frc.robot.subsystems.AlgaeClawSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class GrabAlgae extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final AlgaeClawSubsystem algaeSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public GrabAlgae(AlgaeClawSubsystem algaeSubsystem) {
    this.algaeSubsystem = algaeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(algaeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    algaeSubsystem.grab();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeSubsystem.stow();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return algaeSubsystem.hasAlgae();
  }
}