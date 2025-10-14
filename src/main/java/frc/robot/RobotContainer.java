// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.DASHButtons;
import frc.robot.subsystems.swervedrive.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.ArmSubsystem;
import frc.robot.subsystems.swervedrive.ClimberSubsystem;
import frc.robot.subsystems.swervedrive.ElevatorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision.Cameras;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandJoystick driverJoystick = new CommandJoystick(0);
  final         CommandJoystick DASH = new CommandJoystick(2);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverJoystick.getY() * -1,
                                                                () -> driverJoystick.getX() * -1)
                                                            .withControllerRotationAxis(() -> driverJoystick.getTwist() * -1)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                                                           .withControllerRotationAxis(() -> driverJoystick.getTwist() * -1)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverJoystick.getY(),
                                                                        () -> -driverJoystick.getX())
                                                                    .withControllerRotationAxis(() -> driverJoystick.getTwist() * -1)
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverJoystick.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverJoystick.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

    // arm.setDefaultCommand(arm.setAngle(Degrees.of(0)));
    // elevator.setDefaultCommand(elevator.set(0));

    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    // Safety
    NamedCommands.registerCommand("Safety", arm.setAngle(Degrees.of(50)));

    // Intake
    NamedCommands.registerCommand("Intake", intake.intakeCommand().withTimeout(2));

    // Outtake
    NamedCommands.registerCommand("Outtake", intake.outtakeCommand().withTimeout(2));

    
    NamedCommands.registerCommand("L1Height", elevator.setHeight(Meters.of(0)).withTimeout(3));
    NamedCommands.registerCommand("L1Angle", arm.setAngle(Degrees.of(0)));

    // L2
    NamedCommands.registerCommand("L2Height", elevator.setHeight(Meters.of(1.05)).withTimeout(3));
    NamedCommands.registerCommand("L2Angle", arm.setAngle(Degrees.of(95)).withTimeout(3));

    // L3
    NamedCommands.registerCommand("L3Height", elevator.setHeight(Meters.of(0)).withTimeout(3));
    NamedCommands.registerCommand("L3Angle", arm.setAngle(Degrees.of(0)));

    // AL2
    NamedCommands.registerCommand("AL2Height", elevator.setHeight(Meters.of(0)).withTimeout(3));
    NamedCommands.registerCommand("AL2Angle", arm.setAngle(Degrees.of(0)).withTimeout(3));

    // AL3
    NamedCommands.registerCommand("AL3Height", elevator.setHeight(Meters.of(1.7)).withTimeout(3));
    NamedCommands.registerCommand("AL3Angle", arm.setAngle(Degrees.of(100)).withTimeout(3));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
                                 Rotation2d.fromDegrees(90));
      //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(5, 2)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(Units.degreesToRadians(360),
                                                                                     Units.degreesToRadians(180))
                                           ));
      driverJoystick.button(5).onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverJoystick.button(3).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverJoystick.button(4).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                     () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));



//      driverJoystick.b().whileTrue(
//          drivebase.driveToPose(
//              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
//                              );

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverJoystick.button(3).whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      // driverJoystick.button(10).whileTrue(drivebase.driveToDistanceCommand(10.0, 0.2));
      // driverJoystick.a().whileTrue(drivebase.aimAtTarget(Cameras.CENTER_CAM));
      // driverJoystick.button(4).whileTrue(drivebase.driveToPose(new Pose2d(3.405, 2.497, Rotation2d.fromDegrees(5.194))));
      driverJoystick.button(6).onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverJoystick.button(5).whileTrue(drivebase.centerModulesCommand());
      driverJoystick.button(11).whileTrue(drivebase.sysIdAngleMotorCommand());
      driverJoystick.button(12).whileTrue(drivebase.sysIdDriveMotorCommand());
    } else
    {
      driverJoystick.button(3).onTrue((Commands.runOnce(drivebase::zeroGyro)));
      // driverJoystick.button(0).onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverJoystick.button(4).whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverJoystick.povUp().whileTrue(climber.raiseClimber());
      driverJoystick.povDown().whileTrue(climber.lowerClimber());
      driverJoystick.button(2).onTrue(new ParallelCommandGroup(elevator.setHeight(Meters.of(0)), arm.setAngle(Degrees.of(0))));
      
      // DASH Joystick manually moves both the elevator and the arm
      new Trigger(() -> DASH.getY() > 0.1).whileTrue(elevator.set(0.2));
      new Trigger(() -> DASH.getY() < -0.1).whileTrue(elevator.set(-0.2));
      new Trigger(() -> DASH.getX() > 0.1).whileTrue(arm.set(1));
      new Trigger(() -> DASH.getX() < -0.1).whileTrue(arm.set(-1));

      // Operates the intake
      DASH.button(DASHButtons.INTAKE).whileTrue(intake.intakeCommand());
      DASH.button(DASHButtons.OUTTAKE).whileTrue(intake.outtakeCommand());

      // Coral Button
      DASH.button(DASHButtons.CORAL)
      .onTrue(
        new SequentialCommandGroup(
          arm.setAngle(Degrees.of(45)).withTimeout(2),
        new ParallelCommandGroup(
          elevator.setHeight(Meters.of(0)), 
          arm.setAngle(Degrees.of(100))
        )
        )
      );

      // Algae Button
      DASH.button(DASHButtons.ALGAE)
      .onTrue(
        new SequentialCommandGroup(
          arm.setAngle(Degrees.of(45)).withTimeout(2),
        new ParallelCommandGroup(
          elevator.setHeight(Meters.of(0.32)), 
          arm.setAngle(Degrees.of(97))
        )
        )
      );

      // Processor Button
      DASH.button(DASHButtons.PROCESSOR)
      .onTrue(
        new SequentialCommandGroup(
          arm.setAngle(Degrees.of(45)).withTimeout(2),
        new ParallelCommandGroup(
          elevator.setHeight(Meters.of(0.62)), 
          arm.setAngle(Degrees.of(100))
        )
        )
      );

      // EB Button
      // DASH.button(DASHButtons.EB).onTrue(Commands.none());

      // L1
      DASH.button(DASHButtons.L1)
      .onTrue(
        new SequentialCommandGroup(
          arm.setAngle(Degrees.of(45)).withTimeout(2),
        new ParallelCommandGroup(
          elevator.setHeight(Meters.of(0.8)), 
          arm.setAngle(Degrees.of(95))
        )
        )
      );

      // L2
      DASH.button(DASHButtons.L2)
      .onTrue(
        new SequentialCommandGroup(
          arm.setAngle(Degrees.of(45)).withTimeout(2),
        new ParallelCommandGroup(
          elevator.setHeight(Meters.of(1.05)),   
          arm.setAngle(Degrees.of(95))
        )
        )
      );
      
      // L3
      DASH.button(DASHButtons.L3)
      .onTrue(
        new SequentialCommandGroup(
          arm.setAngle(Degrees.of(45)).withTimeout(2),
        new ParallelCommandGroup(
          elevator.setHeight(Meters.of(1.55)), 
          arm.setAngle(Degrees.of(95))
        )
        )
      );

      // AL2
      DASH.button(DASHButtons.AL2)
      .onTrue(
        new SequentialCommandGroup(
          arm.setAngle(Degrees.of(45)).withTimeout(2),
        new ParallelCommandGroup(
          elevator.setHeight(Meters.of(1.35)), 
          arm.setAngle(Degrees.of(105))
        )
        )
      );

      // AL3
      DASH.button(DASHButtons.AL3)
      .onTrue(
        new SequentialCommandGroup(
          arm.setAngle(Degrees.of(45)).withTimeout(2),
        new ParallelCommandGroup(
          elevator.setHeight(Meters.of(1.7)), 
          arm.setAngle(Degrees.of(100))
        )   
        )
      );

    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("Bug Testing");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
