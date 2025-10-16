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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkMax climberLeader;
  private final SparkMax climberFollower;

  /** Creates a new ExampleSubsystem. */
  public ClimberSubsystem() {
    climberLeader = new SparkMax(13, MotorType.kBrushless);

    climberFollower = new SparkMax(14, MotorType.kBrushless);

    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig followerConfig = new SparkMaxConfig();

    globalConfig
      .smartCurrentLimit(40)
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(0.3);

    followerConfig
      .apply(globalConfig)
      .follow(climberLeader)
      .inverted(true);

    climberLeader.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    climberFollower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public final BrakeSubsystem brake = new BrakeSubsystem(
    1, 0.0, 35.0,
    2, 160.0, 130.0
  );

  public void setSpeed(double speed) {
    climberLeader.set(speed);
  }

  public void stop() {
    climberLeader.set(0);
  }


  public Command raiseClimber() {
    return Commands.run(() -> this.setSpeed(1.0), this)
        .finallyDo(() -> this.stop());
  }

  public Command lowerClimber() {
    return Commands.run(() -> this.setSpeed(-1.0), this)
        .finallyDo(() -> this.stop());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
