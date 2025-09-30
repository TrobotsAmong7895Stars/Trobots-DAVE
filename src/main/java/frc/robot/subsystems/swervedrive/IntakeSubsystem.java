// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax intakeLeader;
  private final SparkMax intakeFollower;
  // private Debouncer currentDebounce = new Debouncer(0.1, Debouncer.DebounceType.kRising);

  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {
    intakeLeader = new SparkMax(10, MotorType.kBrushed);

    intakeFollower = new SparkMax(11, MotorType.kBrushed);

    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig followerConfig = new SparkMaxConfig();

    globalConfig
      .smartCurrentLimit(40)
      .idleMode(IdleMode.kBrake);

    followerConfig
      .apply(globalConfig)
      .follow(intakeLeader)
      .inverted(true);

    intakeLeader.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeFollower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setSpeed(double speed) {
    intakeLeader.set(speed);
  }

  public void stop() {
    intakeLeader.set(0);
  }

  /*
  public boolean isGamePieceIn() {
    return currentDebounce.calculate(intakeLeader.getOutputCurrent() >= 40.0);
  }
    */

  public Command intakeCommand() {
    return Commands.run(() -> this.setSpeed(-1.0)).finallyDo(() -> this.stop());
  }

  public Command outtakeCommand() {
    return Commands.run(() -> this.setSpeed(1.0)).finallyDo(() -> this.stop());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putBoolean("Has Game Piece", isGamePieceIn());
  }
}
