// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BrakeSubsystem extends SubsystemBase {
  private static class Brake {
    private Servo servo;
    private double disengaged;
    private double engaged;
    private boolean isEngaged = false;

    Brake (int port, double disengaged, double engaged) {
      this.servo = new Servo(port);
      this.disengaged = disengaged;
      this.engaged = engaged;
    }
  }

  private final List<Brake> brakes = new ArrayList<>();

/**
   * Constructor to support multiple brakes.
   * @param configs
   */
  public BrakeSubsystem(double... configs) {
    if (configs.length % 3 != 0) {
      throw new IllegalArgumentException("Configs must be in groups of 3: port, disengagedAngle, engagedAngle");
    }
    for (int i = 0; i < configs.length; i += 3) {
      int port = (int) configs[i];
      double disengagedAngle = configs[i + 1];
      double engagedAngle = configs[i + 2];
      brakes.add(new Brake(port, disengagedAngle, engagedAngle));
    }
  }

/**
   * Returns true if the specified brake is engaged (on).
   * @param index
   */
  public boolean isBrakeEngaged(int index) {
    return brakes.get(index).isEngaged;
  }

  private void disengageBrake(int index) {
    Brake brake = brakes.get(index);
    brake.servo.setAngle(brake.disengaged);
    brake.isEngaged = false;
  }

  private void engageBrake(int index) {
    Brake brake = brakes.get(index);
    brake.servo.setAngle(brake.engaged);
    brake.isEngaged = true;
  }

  private void disengageAll() {
    for (Brake brake : brakes) {
      brake.servo.setAngle(brake.disengaged);
      brake.isEngaged = false;
    }
  }

  private void engageAll() {
    for (Brake brake : brakes) {
      brake.servo.setAngle(brake.engaged);
      brake.isEngaged = true;
    }
  }

/**
   * Command to disengage the specified brake (off).
   * @param index
   */
  public Command disengageCommand(int index) {
    return Commands.runOnce(() -> disengageBrake(index), this)
        .andThen(Commands.waitSeconds(0.2));
  }

  /**
   * Command to engage the specified brake (on).
   * @param index
   */
  public Command engageCommand(int index) {
    return Commands.runOnce(() -> engageBrake(index), this)
        .andThen(Commands.waitSeconds(0.2));
  }

/**
   * Command to disengage all brakes.
   */
  public Command disengageAllCommand() {
    return Commands.runOnce(this::disengageAll, this)
        .andThen(Commands.waitSeconds(0.2));
  }

  /**
   * Command to engage all brakes.
   */
  public Command engageAllCommand() {
    return Commands.runOnce(this::engageAll, this)
        .andThen(Commands.waitSeconds(0.2));
  }

  /**
   * Returns true if all brakes are engaged.
   */
  public boolean areAllBrakesEngaged() {
    return brakes.stream().allMatch(b -> b.isEngaged);
  }

  @Override
  public void periodic() {
    for (int i = 0; i < brakes.size(); i++) {
      Brake brake = brakes.get(i);
      SmartDashboard.putBoolean("Brake " + i + " Engaged", brake.isEngaged);
      SmartDashboard.putNumber("Brake " + i + " Angle (degrees)", brake.servo.getAngle());
    }
  }
}
