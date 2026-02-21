// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

/**
 * ============================================================================
 *                    TEAM 3020 - INTAKE ROLLERS
 * ============================================================================
 *
 * This is the intake roller! It's a Kraken X44 motor that spins to grab
 * game pieces from the floor (or spit them out if you grabbed the wrong one).
 *
 * HOW IT WORKS:
 *   - Triangle = Run intake rollers only (without moving the arm)
 *   - Square   = Auto intake (arm + rollers together - use this one!)
 *   - Circle   = Eject (spit out game piece)
 *
 * MOTOR:
 *   - CAN ID 5 = Kraken X44
 *
 * TUNING:
 *   Check Constants.java for:
 *   - kIntakePowerPercent = How fast it sucks in (default 75%)
 *   - kIntakeReversePowerPercent = How fast it spits out (default 50%)
 *
 * @author Baichen Yu
 * ============================================================================
 */
public class IntakeSubsystem extends SubsystemBase {

  // The intake motor
  private final TalonFX m_motor;
  private final DutyCycleOut m_dutyCycle = new DutyCycleOut(0).withEnableFOC(true);

  // State tracking
  private boolean m_isRunning = false;
  private double m_currentPower = 0.0;

  // Stats
  private double m_totalRotations = 0.0;
  private double m_peakCurrent = 0.0;
  private double m_sessionStartTime = 0.0;

  // Power management (optional - reduces power when battery is low)
  private PowerManagementSubsystem m_powerManagement = null;

  /**
   * Creates the intake subsystem.
   */
  public IntakeSubsystem() {
    System.out.println("[INTAKE] Starting up...");

    // Connect to the motor
    m_motor = new TalonFX(MotorConstants.kMotorGroup3MotorID);
    configureMotor();

    m_sessionStartTime = Timer.getFPGATimestamp();

    System.out.println("[INTAKE] Roller ready - CAN ID " + MotorConstants.kMotorGroup3MotorID);
    System.out.println("[INTAKE] Intake power: " + MotorConstants.kIntakePowerPercent + "%");
  }

  /**
   * Hook up power management (optional).
   * When battery is low, intake will slow down to save power.
   */
  public void setPowerManagement(PowerManagementSubsystem pm) {
    m_powerManagement = pm;
  }

  /**
   * Configures the motor with current limits and output settings.
   */
  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Current limiting for Kraken X44
    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.SupplyCurrentLimit = MotorConstants.kKraken44CurrentLimit;
    currentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits = currentLimits;

    // Motor output settings
    MotorOutputConfigs motorOutput = new MotorOutputConfigs();
    motorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorOutput.NeutralMode = NeutralModeValue.Brake;  // Hold position when stopped
    config.MotorOutput = motorOutput;

    // Quick ramp for responsive intake
    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;

    // Apply config
    for (int i = 0; i < 3; i++) {
      if (m_motor.getConfigurator().apply(config).isOK()) break;
    }
  }

  // ========== COMMANDS CALL THESE ==========

  /**
   * Run intake to grab game pieces.
   * Spins the roller to suck in.
   */
  public void runIntake() {
    double power = MotorConstants.kIntakePowerPercent / 100.0;

    // Reduce power if battery is low
    if (m_powerManagement != null) {
      power *= m_powerManagement.getIntakePowerScale();
    }

    m_currentPower = power;
    m_motor.setControl(m_dutyCycle.withOutput(power));
    m_isRunning = true;
    SmartDashboard.putString("Intake Status", "SUCKING @ " + (int)(power * 100) + "%");
  }

  /**
   * Run intake backwards to eject game pieces.
   * Use this if you grabbed the wrong one!
   */
  public void runReverse() {
    double power = -MotorConstants.kIntakeReversePowerPercent / 100.0;

    // Reduce power if battery is low
    if (m_powerManagement != null) {
      power *= m_powerManagement.getIntakePowerScale();
    }

    m_currentPower = power;
    m_motor.setControl(m_dutyCycle.withOutput(power));
    m_isRunning = true;
    SmartDashboard.putString("Intake Status", "EJECTING @ " + (int)(Math.abs(power) * 100) + "%");
  }

  /**
   * Run at a specific power level.
   * Positive = intake, negative = eject.
   */
  public void runAtPower(double power) {
    m_currentPower = Math.max(-1.0, Math.min(1.0, power));
    m_motor.setControl(m_dutyCycle.withOutput(m_currentPower));
    m_isRunning = Math.abs(power) > 0.01;
  }

  /**
   * Stop the intake roller.
   */
  public void stop() {
    m_motor.setControl(m_dutyCycle.withOutput(0));
    m_isRunning = false;
    m_currentPower = 0.0;
    SmartDashboard.putString("Intake Status", "STOPPED");
  }

  // ========== INFO GETTERS ==========

  /** Is the intake currently running? */
  public boolean isRunning() {
    return m_isRunning;
  }

  /** Current amps being drawn */
  public double getCurrentAmps() {
    return m_motor.getSupplyCurrent().getValueAsDouble();
  }

  /** Current RPM of the roller */
  public double getRPM() {
    return m_motor.getVelocity().getValueAsDouble() * 60.0;
  }

  /** Motor temperature in Celsius */
  public double getMotorTemp() {
    return m_motor.getDeviceTemp().getValueAsDouble();
  }

  /** Total rotations since startup */
  public double getTotalRotations() {
    return m_totalRotations;
  }

  /** Highest current we've drawn */
  public double getPeakCurrent() {
    return m_peakCurrent;
  }

  // ========== PERIODIC - RUNS EVERY 20MS ==========

  @Override
  public void periodic() {
    double amps = getCurrentAmps();
    double rpm = getRPM();

    // Track stats
    m_totalRotations += Math.abs(m_motor.getVelocity().getValueAsDouble()) * 0.02;
    if (amps > m_peakCurrent) m_peakCurrent = amps;

    // Update SmartDashboard
    SmartDashboard.putNumber("Intake RPM", rpm);
    SmartDashboard.putNumber("Intake Amps", amps);
    SmartDashboard.putNumber("Intake Temp C", getMotorTemp());
    SmartDashboard.putNumber("Intake Power %", m_currentPower * 100);
    SmartDashboard.putBoolean("Intake Running", m_isRunning);
    SmartDashboard.putBoolean("Intake Alive", m_motor.isAlive());
  }
}
