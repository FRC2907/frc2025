// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.CoralPoop;
import frc.robot.commands.EndPoop;
import frc.robot.constants.Control;
import frc.robot.constants.Ports;
import frc.robot.subsystems.AlgaeClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoopSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.PS5Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final DriveSubsystem driveSubsystem;
  private final PoopSubsystem poopSubsystem;
  //private final AlgaeClawSubsystem algaeClawSubsystem;


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final PS5Controller driver = new PS5Controller(Ports.HID.DRIVER);
  private final PS5Controller operator = new PS5Controller(Ports.HID.OPERATOR);

  private static final SlewRateLimiter xLimiter = new SlewRateLimiter(10);
  private static final SlewRateLimiter yLimiter = new SlewRateLimiter(10);
  private static final SlewRateLimiter rotLimiter = new SlewRateLimiter(10);

  private PathPlannerAuto test;

  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveSubsystem = new DriveSubsystem();
    poopSubsystem = new PoopSubsystem();
    //algaeClawSubsystem = new AlgaeClawSubsystem();

    NamedCommands.registerCommand("Coral Poop", new CoralPoop(poopSubsystem));
    NamedCommands.registerCommand("End Poop", new EndPoop(poopSubsystem));

    test = new PathPlannerAuto("Test");

    autoChooser = AutoBuilder.buildAutoChooser();

    /*RunCommand drive = new RunCommand(
      () -> {
        if (Util.checkDriverDeadband(driver)) 
          driveSubsystem.drive(
            - yLimiter.calculate(driver.getLeftY()) * Control.drivetrain.kMaxVelMeters,
            - xLimiter.calculate(driver.getLeftX()) * Control.drivetrain.kMaxVelMeters,
            - rotLimiter.calculate(driver.getRightX()) * Control.drivetrain.kMaxAngularVel,
            true);
        else 
            driveSubsystem.stop(); },
            driveSubsystem
      );*/

    driveSubsystem.setDefaultCommand(new RunCommand(
      () -> {
        if (Util.checkDriverDeadband(driver)) 
          driveSubsystem.drive(
            - yLimiter.calculate(driver.getLeftY()) * Control.drivetrain.kMaxVelMeters,
            - xLimiter.calculate(driver.getLeftX()) * Control.drivetrain.kMaxVelMeters,
            - rotLimiter.calculate(driver.getRightX()) * Control.drivetrain.kMaxAngularVel,
            false);
        else 
            driveSubsystem.stop(); },
            driveSubsystem
    ));

    configureBindings();

    //PathfindingCommand.warmupCommand().schedule();
    // Configure the trigger bindings
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new JoystickButton(driver, Button.kR2.value).whileTrue(new CoralPoop(poopSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
