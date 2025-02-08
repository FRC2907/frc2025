// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.constants.Control;
import frc.robot.subsystems.DriveSubsystem;

import java.util.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class DriveToPose extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem driveSubsystem;
  private List<Pose2d> goal;
  private Rotation2d endState;
  private PathPlannerPath path;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveToPose(DriveSubsystem subsystem, List<Pose2d> goal, Rotation2d endState) {
    driveSubsystem = subsystem;
    this.goal = goal;
    this.endState = endState;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    List<Pose2d> pathPoses = new Stack<Pose2d>();
    pathPoses.add(driveSubsystem.getPose2d());
    for (int i = 0; i < goal.size(); i++){
      pathPoses.add(goal.get(i));
    }
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      pathPoses
    );

    path = new PathPlannerPath(
      waypoints,
      Control.drivetrain.pathConstraints, 
      null,
      new GoalEndState(0.0, endState)
    );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    AutoBuilder.followPath(path);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
