// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * ============================================================================
 *                    TEAM 3020 - SIMPLE TELEMETRY
 * ============================================================================
 *
 * This subsystem keeps track of basic robot info and puts it on the dashboard.
 * Nothing fancy - just the stuff you actually need during a match.
 *
 * WHAT IT SHOWS:
 *   - Battery voltage (so you know when to swap batteries)
 *   - Match time (how long you've been running)
 *   - Loop time (to catch if code is running slow)
 *
 * That's it! Simple and reliable.
 *
 * @author Baichen Yu
 * ============================================================================
 */
public class TelemetrySubsystem extends SubsystemBase {

  // When did we start?
  private double m_startTime;
  private double m_lastLoopTime;
  private int m_loopCount;

  // Track lowest battery voltage (useful for knowing if battery is weak)
  private double m_minVoltage = 13.0;

  /**
   * Creates the telemetry subsystem.
   * Just sets up the start time - nothing complicated.
   */
  public TelemetrySubsystem() {
    m_startTime = Timer.getFPGATimestamp();
    m_lastLoopTime = m_startTime;
    m_loopCount = 0;

    System.out.println("============================================");
    System.out.println("    TEAM 3020 TELEMETRY READY");
    System.out.println("============================================");
  }

  /**
   * Runs every 20ms. Updates the dashboard with current info.
   */
  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();
    m_loopCount++;

    // Calculate how long this loop took (should be ~20ms)
    double loopTimeMs = (now - m_lastLoopTime) * 1000.0;
    m_lastLoopTime = now;

    // Get battery voltage
    double voltage = RobotController.getBatteryVoltage();
    if (voltage < m_minVoltage) {
      m_minVoltage = voltage;
    }

    // Calculate uptime
    double uptimeSeconds = now - m_startTime;

    // Put everything on SmartDashboard
    SmartDashboard.putNumber("Battery Volts", voltage);
    SmartDashboard.putNumber("Battery Min", m_minVoltage);
    SmartDashboard.putNumber("Uptime (s)", uptimeSeconds);
    SmartDashboard.putNumber("Loop Time (ms)", loopTimeMs);

    // Warn if battery is getting low
    if (voltage < 11.5) {
      SmartDashboard.putString("Battery Status", "LOW - SWAP SOON!");
    } else if (voltage < 12.0) {
      SmartDashboard.putString("Battery Status", "Getting Low");
    } else {
      SmartDashboard.putString("Battery Status", "Good");
    }

    // Warn if loop is running slow (should be under 25ms normally)
    if (loopTimeMs > 30) {
      SmartDashboard.putBoolean("Code Running Slow", true);
    } else {
      SmartDashboard.putBoolean("Code Running Slow", false);
    }
  }

  // ========== GETTERS ==========
  // Use these if other code needs this info

  /** How long has the robot been running? (seconds) */
  public double getUptime() {
    return Timer.getFPGATimestamp() - m_startTime;
  }

  /** Current battery voltage */
  public double getBatteryVoltage() {
    return RobotController.getBatteryVoltage();
  }

  /** Lowest voltage we've seen this session */
  public double getMinVoltage() {
    return m_minVoltage;
  }

  /** How many loops have we run? */
  public int getLoopCount() {
    return m_loopCount;
  }
}
