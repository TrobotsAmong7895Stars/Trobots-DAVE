// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;

import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConfig;

public class ArmSubsystem extends SubsystemBase {

  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final PIDController pidController;
  private final ArmFeedforward feedforward;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    motor = new SparkMax(12, MotorType.kBrushless);

    SparkMaxConfig armConfig = new SparkMaxConfig();

    armConfig
      .smartCurrentLimit(20)
      .inverted(false)
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(0.8);

    armConfig.encoder
      .positionConversionFactor(ArmConfig.POSITION_CONVERSION_FACTOR)
      .velocityConversionFactor(ArmConfig.VELOCITY_CONVERSION_FACTOR);

    motor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = motor.getEncoder();
    encoder.setPosition(0.0);

    pidController = new PIDController(ArmConfig.KP, ArmConfig.KI, ArmConfig.KD);
    pidController.setTolerance(ArmConfig.POSITION_TOLERANCE_DEG);

    feedforward = new ArmFeedforward(ArmConfig.KS, ArmConfig.KG, ArmConfig.KV, ArmConfig.KA);
  }

  /**
   * Command to set the arm to a specific angle using closed-loop control.
   * This command completes immediately, but the subsystem will continue to move and hold the position.
   *
   * @param angle Target angle in degrees.
   * @return A command to set the target.
   */
  public Command setAngle(Angle angle) {
    return runOnce(() -> {
      ArmConfig.isManualMode = false;
      pidController.reset();
      ArmConfig.setpoint = angle.in(Degrees);
    });
  }

  /**
   * Command for manual open-loop control of the arm.
   * This runs continuously until canceled, overriding closed-loop control.
   *
   * @param dutyCycle Speed in [-1, 1].
   * @return A command to control the arm manually.
   */
  public Command set(double dutyCycle) {
    return run(() -> {
      ArmConfig.isManualMode = true;
      motor.set(dutyCycle);
    }).finallyDo(() -> {
      ArmConfig.isManualMode = false;
      motor.set(0.0);
    });
  }

  /**
   * Gets the current arm angle in degrees.
   *
   * @return Current angle.
   */
  public double getAngle() {
    return encoder.getPosition();
  }

  /**
   * Checks if the arm is at the target position (within tolerance).
   *
   * @return True if at target.
   */
  public boolean isAtTarget() {
    return pidController.atSetpoint();
  }

  @Override
  public void periodic() {
    if (!ArmConfig.isManualMode) {
      double pidOutput = pidController.calculate(getAngle(), ArmConfig.setpoint);
      double ffOutput = feedforward.calculate(Math.toRadians(getAngle()), encoder.getVelocity());
      motor.setVoltage(pidOutput + ffOutput);
    }

    SmartDashboard.putNumber("Arm Angle (deg)", getAngle());
    SmartDashboard.putNumber("Arm Target (deg)", ArmConfig.setpoint);
    SmartDashboard.putBoolean("Arm At Target", isAtTarget());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}