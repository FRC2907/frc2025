// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Stow;
import frc.robot.commands.drive.ReefLeft;
import frc.robot.commands.drive.ReefNearest;
import frc.robot.commands.drive.ReefRight;
import frc.robot.commands.elevator.ElevatorDown;
import frc.robot.commands.elevator.ElevatorUp;
import frc.robot.commands.grabAlgae.GrabAlgae;
import frc.robot.commands.grabAlgae.GrabAlgaeGround;
import frc.robot.commands.processor.ProcessorPrep;
import frc.robot.constants.*;
import frc.robot.subsystems.*;
import frc.robot.util.Util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.PS5Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  //private final PoopSubsystem poopSubsystem;
  //private final AlgaeClawSubsystem algaeClawSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;

  private RunCommand drive, lockDrive, danceDriveRight, danceDriveLeft;

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
    //poopSubsystem = PoopSubsystem.getInstance();
    //algaeClawSubsystem = AlgaeClawSubsystem.getInstance();
    elevatorSubsystem = ElevatorSubsystem.getInstance();
    
    /*NamedCommands.registerCommand("Coral Poop", new CoralPoop(poopSubsystem));
    NamedCommands.registerCommand("Wait Coral Intake", poopSubsystem.coralWaitIntakeCommand());
    NamedCommands.registerCommand("Wait Coral Shoot", poopSubsystem.coralWaitShootCommand());*/

    //NamedCommands.registerCommand("Coral Station", elevatorSubsystem.coralStationCommand());
    //NamedCommands.registerCommand("L1", elevatorSubsystem.L1Command());
    //NamedCommands.registerCommand("L2", elevatorSubsystem.L2Command());
    //NamedCommands.registerCommand("L3", elevatorSubsystem.L3Command());


    autoChooser = AutoBuilder.buildAutoChooser();

    kindaUgly();
    driveSubsystem.setDefaultCommand(drive);
    //algaeClawSubsystem.setDefaultCommand(algaeClawSubsystem.stowCommand());

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
    /*
     * ALL DRIVER BINDINGS
     */
    //new Trigger(() -> Util.checkPOVLeft(driver)).onTrue(new ReefLeft(driveSubsystem));
    //new Trigger(() -> Util.checkPOVRight(driver)).onTrue(new ReefRight(driveSubsystem));
    //new JoystickButton(driver, Button.kL2.value).onTrue(new ReefNearest(driveSubsystem));
    new JoystickButton(driver, Button.kL1.value).whileTrue(danceDriveLeft);
    new JoystickButton(driver, Button.kR1.value).whileTrue(danceDriveRight);
    new JoystickButton(driver, Button.kR2.value).whileTrue(lockDrive);
    // should create a do-nothing command that requires the driveSubsystem, causing existing commands using that system to be cancelled?
    new JoystickButton(driver, Button.kCircle.value).onTrue(new InstantCommand(() -> {}, driveSubsystem));



    /*
     * ALL OPERATOR BINDINGS
     */
    /*new JoystickButton(operator, Button.kL1.value).whileTrue(algaeClawSubsystem.algaePoop());
    new JoystickButton(operator, Button.kCross.value).whileTrue(algaeClawSubsystem.testIntake());
    /*new JoystickButton(operator, Button.kCross.value).onTrue(algaeClawSubsystem.intakeAlgae());
    new JoystickButton(operator, Button.kCircle.value).onTrue(new ProcessorPrep(algaeClawSubsystem, elevatorSubsystem));
    new JoystickButton(operator, Button.kR2.value).onTrue(algaeClawSubsystem.shootPrep());
    new JoystickButton(operator, Button.kSquare.value).onTrue(algaeClawSubsystem.shoot());
    new JoystickButton(operator, Button.kL2.value).onTrue(new GrabAlgae(algaeClawSubsystem));  
    new JoystickButton(operator, Button.kCircle.value).whileTrue(algaeClawSubsystem.feedforwardTest());
    new JoystickButton(operator, Button.kCreate.value).onTrue(new GrabAlgaeGround(algaeClawSubsystem));
    new JoystickButton(operator, Button.kR2.value).whileTrue(algaeClawSubsystem.testPID());
    new JoystickButton(operator, Button.kL2.value).onTrue(new GrabAlgae(algaeClawSubsystem));
    new Trigger(() -> Util.checkRightJoystickControl(operator, false)) .whileTrue(algaeClawSubsystem.testUp());
    new Trigger(() -> Util.checkRightJoystickControl(operator, true)).whileTrue(algaeClawSubsystem.testDown());

    //new JoystickButton(operator, Button.kCircle.value).onTrue(elevatorSubsystem.reset());
    new JoystickButton(operator, Button.kTriangle.value).onTrue(elevatorSubsystem.coralStationCommand());
    new Trigger(() -> Util.checkPOVUp(operator)).toggleOnTrue(
      new InstantCommand(() -> new ElevatorUp(elevatorSubsystem).schedule()));
    new Trigger(() -> Util.checkPOVDown(operator)).toggleOnTrue(
      new InstantCommand(() -> new ElevatorDown(elevatorSubsystem).schedule()));
    new Trigger(() -> Util.checkPOVLeft(operator) || Util.checkPOVRight(operator)).onTrue(
      new InstantCommand(() -> elevatorSubsystem.switchScoring(), elevatorSubsystem)
    );*/
    new Trigger(() -> Util.checkLeftJoystickControl(operator, true)) .whileTrue(elevatorSubsystem.manualDown());
    new Trigger(() -> Util.checkLeftJoystickControl(operator, false)).whileTrue(elevatorSubsystem.manualUp());

    //new JoystickButton(operator, Button.kR1.value).whileTrue(poopSubsystem.coralPoop());


    /*new JoystickButton(operator, Button.kPS.value).onTrue(
      new InstantCommand(() -> {}, algaeClawSubsystem, elevatorSubsystem, poopSubsystem));*/


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

  public void disabledPeriodic(){
    driveSubsystem.stop();
  }

  private void kindaUgly(){
    danceDriveLeft = new RunCommand(
      () -> {
        driveSubsystem.drive(
          - yLimiter.calculate(driver.getLeftY()),
          - xLimiter.calculate(driver.getLeftX()),
          - Control.drivetrain.kSpinSpeed,
          false);
      }, driveSubsystem
    );
    danceDriveRight = new RunCommand(
      () -> {
        driveSubsystem.drive(
          - yLimiter.calculate(driver.getLeftY()),
          - xLimiter.calculate(driver.getLeftX()),
          Control.drivetrain.kSpinSpeed,
          false);
      }, driveSubsystem
    );
    lockDrive = new RunCommand(
      () -> {
        driveSubsystem.lockDrive(
          - yLimiter.calculate(driver.getLeftY()) * Control.drivetrain.kMaxVelMPS,
          - xLimiter.calculate(driver.getLeftX()) * Control.drivetrain.kMaxVelMPS, 
          FieldElements.Reef.centerFaces[4],
          true); },
          driveSubsystem 
    );
    drive = new RunCommand(
      () -> {
          driveSubsystem.drive(
            - yLimiter.calculate(MathUtil.applyDeadband(driver.getLeftY(), 0.1)) * Control.drivetrain.kMaxVelMPS,
            - xLimiter.calculate(MathUtil.applyDeadband(driver.getLeftX(), 0.1)) * Control.drivetrain.kMaxVelMPS,
            - rotLimiter.calculate(MathUtil.applyDeadband(driver.getRightX(), 0.1)) * Control.drivetrain.kMaxAngularVelRad, 
            false); }, 
            driveSubsystem
    );
  }
}