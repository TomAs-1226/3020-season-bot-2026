// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PowerConstants;

/**
 * ============================================================================
 *                    TEAM 3020 - POWER MANAGEMENT
 * ============================================================================
 *
 * This subsystem watches the battery and warns you when it's getting low.
 * It runs in the background - you shouldn't notice it until your battery
 * really needs to be swapped.
 *
 * POWER STATES (based on battery voltage):
 *   NOMINAL   = Above 11.5V - Everything's good!
 *   WARNING   = 11.0-11.5V  - Battery is getting low
 *   CRITICAL  = 10.5-11.0V  - Swap the battery soon
 *   EMERGENCY = Below 10.5V - About to brown out!
 *
 * WHAT GETS THROTTLED:
 *   - NOTHING important! The shooter and feeder always run at full power
 *   - Only the intake slows down in EMERGENCY to save power
 *
 * WHY THIS MATTERS:
 *   If battery voltage drops too low, the robot will "brown out" and
 *   reboot. That's really bad during a match! This subsystem helps
 *   prevent that by warning you and reducing power draw.
 *
 * @author Baichen Yu
 * ============================================================================
 */
public class PowerManagementSubsystem extends SubsystemBase {

  // The four power states
  public enum PowerState {
    NOMINAL,    // >11.5V - all systems go
    WARNING,    // 11.0-11.5V - keep an eye on it
    CRITICAL,   // 10.5-11.0V - battery is getting low
    EMERGENCY   // <10.5V - danger zone!
  }

  // Current state
  private PowerState m_currentState = PowerState.NOMINAL;
  private PowerState m_previousState = PowerState.NOMINAL;

  // Voltage tracking
  private double m_currentVoltage = 12.5;
  private double m_minVoltageSession = 12.5;  // Lowest voltage we've seen
  private double m_startVoltage = 12.5;       // Voltage when we started
  private double m_startTime = 0.0;

  // Simple voltage history for calculating discharge rate
  private double m_lastVoltage = 12.5;
  private double m_lastTime = 0.0;
  private double m_voltageSlope = 0.0;  // V/s (negative = discharging)

  // Brownout prediction
  private boolean m_brownoutImminent = false;

  // State change counters (useful for post-match analysis)
  private int m_warningCount = 0;
  private int m_criticalCount = 0;
  private int m_emergencyCount = 0;

  /**
   * Creates the power management subsystem.
   * Starts tracking battery voltage immediately.
   */
  public PowerManagementSubsystem() {
    double now = Timer.getFPGATimestamp();
    m_startTime = now;
    m_lastTime = now;
    m_startVoltage = RobotController.getBatteryVoltage();
    m_lastVoltage = m_startVoltage;
    m_minVoltageSession = m_startVoltage;

    System.out.println("============================================");
    System.out.println("    TEAM 3020 POWER MANAGEMENT READY");
    System.out.println("============================================");
    System.out.println("[POWER] Start voltage: " + String.format("%.2f", m_startVoltage) + "V");
    System.out.println("[POWER] Critical threshold: " + PowerConstants.kCriticalVoltage + "V");
  }

  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();
    m_currentVoltage = RobotController.getBatteryVoltage();

    // Track minimum voltage
    if (m_currentVoltage < m_minVoltageSession) {
      m_minVoltageSession = m_currentVoltage;
    }

    // Calculate voltage slope (how fast is voltage dropping?)
    double dt = now - m_lastTime;
    if (dt > 0.1) {  // Update every 100ms
      m_voltageSlope = (m_currentVoltage - m_lastVoltage) / dt;
      m_lastVoltage = m_currentVoltage;
      m_lastTime = now;
    }

    // Predict brownout
    predictBrownout();

    // Update power state
    updatePowerState();

    // Log state changes
    if (m_currentState != m_previousState) {
      System.out.println("[POWER] " + m_previousState + " -> " + m_currentState +
          " @ " + String.format("%.2f", m_currentVoltage) + "V");

      if (m_currentState == PowerState.WARNING) m_warningCount++;
      else if (m_currentState == PowerState.CRITICAL) m_criticalCount++;
      else if (m_currentState == PowerState.EMERGENCY) m_emergencyCount++;

      m_previousState = m_currentState;
    }

    // Update SmartDashboard
    SmartDashboard.putNumber("Battery V", m_currentVoltage);
    SmartDashboard.putNumber("Battery Min V", m_minVoltageSession);
    SmartDashboard.putString("Power State", m_currentState.toString());
    SmartDashboard.putBoolean("Brownout Risk", m_brownoutImminent);
    SmartDashboard.putNumber("Battery %", getBatteryHealthPercent());
    SmartDashboard.putBoolean("Safe to Shoot", isSafeToShoot());
    SmartDashboard.putNumber("Discharge V/s", m_voltageSlope);
  }

  /**
   * Checks if we're about to brown out.
   */
  private void predictBrownout() {
    // Don't warn unless voltage is already in WARNING range and dropping fast
    if (m_currentVoltage >= PowerConstants.kWarningVoltage || m_voltageSlope >= -0.3) {
      m_brownoutImminent = false;
      return;
    }

    // Calculate time until we hit emergency voltage
    double voltageToEmergency = m_currentVoltage - PowerConstants.kEmergencyVoltage;
    double timeToBrownout = voltageToEmergency / (-m_voltageSlope);

    // Warn if brownout is likely within 2 seconds
    m_brownoutImminent = timeToBrownout < 2.0;
  }

  /**
   * Updates the power state based on current voltage.
   */
  private void updatePowerState() {
    if (m_currentVoltage > PowerConstants.kWarningVoltage) {
      m_currentState = PowerState.NOMINAL;
    } else if (m_currentVoltage > PowerConstants.kCriticalVoltage) {
      m_currentState = PowerState.WARNING;
    } else if (m_currentVoltage > PowerConstants.kEmergencyVoltage) {
      m_currentState = PowerState.CRITICAL;
    } else {
      m_currentState = PowerState.EMERGENCY;
    }

    // If brownout is imminent, escalate WARNING to CRITICAL
    if (m_brownoutImminent && m_currentState == PowerState.WARNING) {
      m_currentState = PowerState.CRITICAL;
    }
  }

  // ========== GETTERS ==========

  /** Get the current power state */
  public PowerState getPowerState() {
    return m_currentState;
  }

  /** Is everything running normally? */
  public boolean isNominal() {
    return m_currentState == PowerState.NOMINAL;
  }

  /** Should we reduce power to save the battery? */
  public boolean shouldConservePower() {
    return m_currentState == PowerState.CRITICAL || m_currentState == PowerState.EMERGENCY;
  }

  /** Is a brownout likely to happen soon? */
  public boolean isBrownoutImminent() {
    return m_brownoutImminent;
  }

  /** Current battery voltage */
  public double getVoltage() {
    return m_currentVoltage;
  }

  /** How fast is voltage dropping? (V/s, negative = discharging) */
  public double getVoltageSlope() {
    return m_voltageSlope;
  }

  /**
   * Battery health as a percentage.
   * 13V = 100%, 10.5V = 0%
   */
  public double getBatteryHealthPercent() {
    double full = 13.0;
    double empty = PowerConstants.kEmergencyVoltage;
    double percent = (m_currentVoltage - empty) / (full - empty) * 100.0;
    return Math.max(0, Math.min(100, percent));
  }

  /**
   * Is it safe to fire the shooter right now?
   * Returns false if voltage is too low (might cause brownout).
   */
  public boolean isSafeToShoot() {
    return m_currentVoltage > PowerConstants.kCriticalVoltage && !m_brownoutImminent;
  }

  /**
   * How much has voltage dropped since we started?
   */
  public double getVoltageDropSinceStart() {
    return m_startVoltage - m_currentVoltage;
  }

  /**
   * How long have we been running? (seconds)
   */
  public double getSessionDuration() {
    return Timer.getFPGATimestamp() - m_startTime;
  }

  // ========== POWER SCALING ==========
  // These methods tell subsystems how much to reduce their power

  /**
   * How much should the intake slow down?
   * Only reduces power in EMERGENCY to save the battery.
   */
  public double getIntakePowerScale() {
    if (m_currentState == PowerState.EMERGENCY) {
      return PowerConstants.kEmergencyModeIntakeScale;  // 60%
    }
    return 1.0;  // Full power
  }

  /** Shooter always runs at full power */
  public double getShooterPowerScale() {
    return 1.0;
  }

  /** Feeder always runs at full power */
  public double getFeederPowerScale() {
    return 1.0;
  }

  // ========== SESSION STATS ==========

  /**
   * Get session stats for logging.
   */
  public String getSessionStats() {
    return String.format(
        "Min V: %.2fV | Drop: %.2fV | Warnings: %d | Criticals: %d | State: %s",
        m_minVoltageSession, getVoltageDropSinceStart(),
        m_warningCount, m_criticalCount, m_currentState);
  }

  /**
   * Reset all tracking (call between matches).
   */
  public void resetSession() {
    m_warningCount = 0;
    m_criticalCount = 0;
    m_emergencyCount = 0;
    m_minVoltageSession = m_currentVoltage;
    m_startVoltage = m_currentVoltage;
    m_startTime = Timer.getFPGATimestamp();
    System.out.println("[POWER] Session reset");
  }
}
