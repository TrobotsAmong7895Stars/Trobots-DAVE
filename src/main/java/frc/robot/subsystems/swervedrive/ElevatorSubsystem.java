// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Second;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.SmartMechanism;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ElevatorSubsystem extends SubsystemBase {

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  // Mechanism Circumference is the distance traveled by each mechanism rotation converting rotations to meters.
  .withMechanismCircumference(Meters.of(Inches.of(2.638).in(Meters) * 22))
  // Feedback Constants (PID Constants)
  .withClosedLoopController(0.0042004, 0, 0, MetersPerSecond.of(1), MetersPerSecondPerSecond.of(0.75))
  .withSimClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
  // Feedforward Constants
  .withFeedforward(new ElevatorFeedforward(0.24395, 0.42824, 11.372))
  .withSimFeedforward(new ElevatorFeedforward(0, 0, 0))
  // Telemetry name and verbosity level
  .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
  // Gearing from the motor rotor to final shaft.
  // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which corresponds to the gearbox attached to your motor.
  .withGearing(SmartMechanism.gearing(SmartMechanism.gearbox(48).div(3)))
  // Motor properties to prevent over currenting.
  .withMotorInverted(false)
  .withIdleMode(MotorMode.BRAKE)
  .withStatorCurrentLimit(Amps.of(40))
  .withClosedLoopRampRate(Seconds.of(0.25))
  .withOpenLoopRampRate(Seconds.of(0.25));

  // Vendor motor controller object
  private SparkMax spark = new SparkMax(9, MotorType.kBrushless);

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController sparkSmartMotorController = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

  private ElevatorConfig elevconfig = new ElevatorConfig(sparkSmartMotorController)
      .withStartingHeight(Meters.of(0.03))
      .withHardLimits(Meters.of(0), Meters.of(1.75))
      .withTelemetry("Elevator", TelemetryVerbosity.HIGH)
      .withMass(Pounds.of(16));

  // Elevator Mechanism
  private Elevator elevator = new Elevator(elevconfig);

  private final BrakeSubsystem brake = new BrakeSubsystem(
    0, 0.0, 35.0
  );

  /**
   * Set the height of the elevator.
   * @param angle Distance to go to.
   */
  public Command setHeight(Distance height) { 
    Command disengageIfNeeded = Commands.either(
      brake.disengageAllCommand(),
      Commands.none(),
      () -> !brake.areAllBrakesEngaged()
    );
    return disengageIfNeeded
      .andThen(elevator.setHeight(height))
      .andThen(brake.engageCommand(0));
  }

  /**
   * Move the elevator up and down.
   * @param dutycycle [-1, 1] speed to set the elevator too.
   */
  public Command set(double dutycycle) {
    Command disengageIfNeeded = Commands.either(
      brake.disengageAllCommand(),
      Commands.none(),
      () -> !brake.areAllBrakesEngaged()
    );
    return disengageIfNeeded
      .andThen(elevator.set(dutycycle))
      .andThen(brake.engageCommand(0));
  }

  /**
   * Run sysId on the {@link Elevator}
   */
  public Command sysId() { return elevator.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));}

  /** Creates a new ExampleSubsystem. */
  public ElevatorSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevator.updateTelemetry();
    SmartDashboard.putNumber("Elevator Height (m)", elevator.getHeight().in(Meters));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    elevator.simIterate();
  }
}
