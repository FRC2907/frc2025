// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.constants.Control;
import frc.robot.subsystems.DriveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class FollowPath extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem driveSubsystem;
  private PathPlannerPath path;
  private int closestPoseIndex, currentPathIndex;
  private boolean switchPath, right;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FollowPath(DriveSubsystem subsystem, boolean switchPath, boolean right) {
    this.driveSubsystem = subsystem;
    this.switchPath = switchPath;
    this.right = right;
    this.currentPathIndex = 0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ARGGGGHHHHHHHHH11111111");
    path = driveSubsystem.blankPath;
    Pose2d closestPose = new Pose2d(99, 99, Rotation2d.fromDegrees(180));
    closestPoseIndex = 0;
    if (!switchPath){
      System.out.println("ARGGGGHHHHHHHHH222222222222");
      Transform2d[] minusPoses = new Transform2d[12];

      for (int i = 0; i < 12; i++){
        System.out.println("ARGGGGHHHHHHHHH3333333");
        try {
          System.out.println("ARGGGGHHHHHHHHH44444444444");
          minusPoses[i] = driveSubsystem.reefPoses[i].minus(driveSubsystem.getPose2d());
          if (minusPoses[i].getTranslation().getNorm() < closestPose.getTranslation().getNorm()){
            System.out.println("ARGGGGHHHHHHHHH555555555555");
            closestPose = driveSubsystem.reefPoses[i];
            closestPoseIndex = i;
          }
        } catch (Exception e){
          System.out.println("ARGGGGHHHHHHHHH666666666");
          e.printStackTrace();
        }
      }
    }

    System.out.println("ARGGGGHHHHHHHHH777777777");
    currentPathIndex = right ? currentPathIndex - 1 : currentPathIndex + 1;
    if (currentPathIndex < 0){
      System.out.println("ARGGGGHHHHHHHHH888888888");
      currentPathIndex = 11;
    } else if (currentPathIndex > 11){
      System.out.println("ARGGGGHHHHHHHHH9999999999999");
      currentPathIndex = 0;
    }
    try {
      System.out.println("ARGGGGHHHHHHHHH10101010101010101010101010101");
      path = switchPath ? driveSubsystem.getPathFile(currentPathIndex) : driveSubsystem.getPathFile(closestPoseIndex);
    } catch (Exception e){
      System.out.println("ARGGGGHHHHHHHHHELEVENELEVENELEVEN11111111");
      e.printStackTrace();
    }
    System.out.println("ARGGGGHHHHHHHHH121212121212121212");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    AutoBuilder.pathfindThenFollowPath(path, Control.drivetrain.pathConstraints);
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
