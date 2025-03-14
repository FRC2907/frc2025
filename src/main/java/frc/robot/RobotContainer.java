// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.*;

import frc.robot.commands.coral.CoralPoop;
import frc.robot.commands.drive.ReefLeft;
import frc.robot.commands.drive.ReefNearest;
import frc.robot.commands.drive.ReefRight;
import frc.robot.commands.grabAlgae.GrabAlgae1;
import frc.robot.commands.grabAlgae.GrabAlgae2;
import frc.robot.commands.grabAlgae.GrabAlgaeGround;
import frc.robot.constants.Control;
import frc.robot.constants.FieldElements;
import frc.robot.constants.Ports;
import frc.robot.subsystems.AlgaeClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PoopSubsystem;
import frc.robot.util.Util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.PS5Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
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
  //private final ElevatorSubsystem elevatorSubsystem;

  private RunCommand drive;
  private RunCommand lockDrive;
  private RunCommand elevatorUp;
  private RunCommand elevatorDown;
  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final PS5Controller driver = new PS5Controller(Ports.HID.DRIVER);
  public final PS5Controller operator = new PS5Controller(Ports.HID.OPERATOR);

  private static final SlewRateLimiter xLimiter = new SlewRateLimiter(10);
  private static final SlewRateLimiter yLimiter = new SlewRateLimiter(10);
  private static final SlewRateLimiter rotLimiter = new SlewRateLimiter(10);

  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveSubsystem = DriveSubsystem.getInstance();
    poopSubsystem = PoopSubsystem.getInstance();
    //algaeClawSubsystem = AlgaeClawSubsystem.getInstance();
    //elevatorSubsystem = ElevatorSubsystem.getInstance();
    

    
    NamedCommands.registerCommand("Coral Poop", new CoralPoop(poopSubsystem));
    NamedCommands.registerCommand("Wait Coral Intake", poopSubsystem.coralWaitIntakeCommand());
    NamedCommands.registerCommand("Wait Coral Shoot", poopSubsystem.coralWaitShootCommand());

    autoChooser = AutoBuilder.buildAutoChooser();

    lockDrive = new RunCommand(
      () -> {
        driveSubsystem.lockDrive(
          - yLimiter.calculate(driver.getLeftY()) * Control.drivetrain.kMaxVelMPS,
          - xLimiter.calculate(driver.getLeftX()) * Control.drivetrain.kMaxVelMPS, 
          FieldElements.Reef.centerFaces[4],
          true); },
          driveSubsystem );

    drive = new RunCommand(
      () -> {
        //if (Util.checkDriverDeadband(driver)) 
          driveSubsystem.drive(
            - yLimiter.calculate(driver.getLeftY()) * Control.drivetrain.kMaxVelMPS,
            - xLimiter.calculate(driver.getLeftX()) * Control.drivetrain.kMaxVelMPS,
            - rotLimiter.calculate(driver.getRightX()) * Control.drivetrain.kMaxAngularVelRad, 
            true);
        /*else 
            driveSubsystem.stop(); */ }, 
            driveSubsystem
    );
    driveSubsystem.setDefaultCommand(new RunCommand(() -> driveSubsystem.stop()));

    //elevatorUp = new RunCommand(elevatorSubsystem::up);
    //elevatorDown = new RunCommand(elevatorSubsystem::down);

    configureBindings();

    FollowPathCommand.warmupCommand().schedule();

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

    //new JoystickButton(operator, Button.kR2.value).whileTrue(new CoralPoop(poopSubsystem));

    new Trigger(() -> Util.checkDriverDeadband(driver)).onTrue(
      drive.withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    List<Pose2d> something = new Stack<Pose2d>();
    something.add(new Pose2d(3, 3, Rotation2d.kZero));
    something.add(FieldElements.Reef.centerFaces[3]);
    //PathPlannerPath thing = driveSubsystem.generatePath(something, Rotation2d.kZero);
    new Trigger(() -> Util.checkPOVLeft(driver)).onTrue(
      new ReefLeft(driveSubsystem).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    new Trigger(() -> Util.checkPOVRight(driver)).onTrue(
      new ReefRight(driveSubsystem).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    new JoystickButton(driver, Button.kL2.value).onTrue(
      new ReefNearest(driveSubsystem).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    new JoystickButton(driver, Button.kR2.value).whileTrue(lockDrive);
    new JoystickButton(driver, Button.kCircle.value).onTrue(new RunCommand(() ->
        CommandScheduler.getInstance().cancel(driveSubsystem.scheduleReefLeftCommand(),
                                              driveSubsystem.scheduleReefRightCommand(),
                                              driveSubsystem.scheduleReefNearestCommand())));

    //new JoystickButton(driver, Button.kSquare.value).onTrue(new GrabAlgae1(algaeClawSubsystem, elevatorSubsystem));
    //new JoystickButton(driver, Button.kCircle.value).onTrue(new GrabAlgae2(algaeClawSubsystem, elevatorSubsystem));
    //new JoystickButton(driver, Button.kSquare.value).onTrue(new GrabAlgaeGround(algaeClawSubsystem, elevatorSubsystem));

    //new Trigger(() -> Util.checkPOVUp(driver)).onTrue(elevatorUp);
    //new Trigger(() -> Util.checkPOVDown(driver)).onTrue(elevatorDown);


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
