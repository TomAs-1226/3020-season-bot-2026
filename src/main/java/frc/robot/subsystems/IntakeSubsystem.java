// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

/**
 * ============================================================================
 *                    TEAM 3020 - INTAKE ROLLERS
 * ============================================================================
 *
 * Intake roller — Falcon 500 (CAN 45, NO FOC) spins to grab game pieces
 * from the floor or spit them out.
 *
 * CONTROLS:
 *   - Triangle = Run intake rollers only (without moving the arm)
 *   - L2       = Full outtake (intake reverse + feeder reverse)
 *   - Circle   = Eject
 *
 * MOTOR:
 *   - CAN ID 45 = Falcon 500 (withEnableFOC(false))
 *
 * TUNING (Shuffleboard "Intake" tab):
 *   - Intake Power %         = How fast it sucks in (default 90%)
 *   - Intake Reverse Power % = How fast it spits out (default 50%)
 *
 * @author Baichen Yu
 * ============================================================================
 */
public class IntakeSubsystem extends SubsystemBase {

  // The intake motor (Falcon 500, CAN 45 - Falcon does NOT support FOC)
  // Using VoltageOut instead of DutyCycleOut — Falcon 500 responds to voltage
  // commands but not duty cycle (confirmed: works in Tuner with velocity/voltage)
  private static final double kNominalVoltage = 12.0;
  private final TalonFX m_motor;
  private final VoltageOut m_voltageOut = new VoltageOut(0).withEnableFOC(false);

  // State tracking
  private boolean m_isRunning = false;
  private double m_currentPower = 0.0;

  // Stats
  private double m_totalRotations = 0.0;
  private double m_peakCurrent = 0.0;

  // Power management (optional - reduces power when battery is low)
  private PowerManagementSubsystem m_powerManagement = null;

  // Preferences save throttle — writing every 20ms causes loop overruns (31ms+)
  private double m_lastPreferencesSaveTime = 0.0;

  // Throttle console prints to once per second (prevent Driver Station log spam)
  private double m_lastLogTime = 0.0;

  // ===== SHUFFLEBOARD — "Intake" tab =====
  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Intake");

  // ----- Tunable inputs (editable in Shuffleboard) -----
  private final GenericEntry m_entryPowerPercent =
      m_tab.add("Intake Power %", MotorConstants.kIntakePowerPercent).getEntry();
  private final GenericEntry m_entryReversePowerPercent =
      m_tab.add("Intake Reverse Power %", MotorConstants.kIntakeReversePowerPercent).getEntry();

  // ----- Status / telemetry (read-only in Shuffleboard) -----
  private final GenericEntry m_entryRPM          = m_tab.add("RPM",           0.0).getEntry();
  private final GenericEntry m_entryAmps         = m_tab.add("Amps",          0.0).getEntry();
  private final GenericEntry m_entryTempC        = m_tab.add("Temp C",        0.0).getEntry();
  private final GenericEntry m_entryAppliedPct   = m_tab.add("Applied %",     0.0).getEntry();
  private final GenericEntry m_entryVoltage      = m_tab.add("Bus Voltage",   0.0).getEntry();
  private final GenericEntry m_entryRunning      = m_tab.add("Running",       false).getEntry();
  private final GenericEntry m_entryAlive        = m_tab.add("Alive",         false).getEntry();
  private final GenericEntry m_entrySetControl   = m_tab.add("setControl",    "N/A").getEntry();
  private final GenericEntry m_entryStatus       = m_tab.add("Status",        "STOPPED").getEntry();

  /**
   * Creates the intake subsystem.
   */
  public IntakeSubsystem() {
    System.out.println("[INTAKE] Starting up...");

    // Connect to the motor on RIO CAN bus
    m_motor = new TalonFX(MotorConstants.kMotorGroup3MotorID, MotorConstants.kMotorGroup3CANBus);
    configureMotor();

    // Load saved values from Preferences into Shuffleboard entries
    m_entryPowerPercent.setDouble(
        Preferences.getDouble("Intake Power %", MotorConstants.kIntakePowerPercent));
    m_entryReversePowerPercent.setDouble(
        Preferences.getDouble("Intake Reverse Power %", MotorConstants.kIntakeReversePowerPercent));

    System.out.println("[INTAKE] Roller ready - CAN ID " + MotorConstants.kMotorGroup3MotorID
        + " on bus \"" + MotorConstants.kMotorGroup3CANBus + "\"");
    System.out.println("[INTAKE] Default power: " + MotorConstants.kIntakePowerPercent + "%");
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

    // Current limiting for Falcon 500
    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.SupplyCurrentLimit = MotorConstants.kFalconCurrentLimit;
    currentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits = currentLimits;

    // Motor output settings
    MotorOutputConfigs motorOutput = new MotorOutputConfigs();
    motorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput = motorOutput;

    // Quick ramp for responsive intake (must match VoltageOut control mode)
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;

    // CRITICAL: Disable hardware limit switches — Falcon 500 data port pins float
    // when nothing is connected. Phoenix 6 enables limits by default, so a floating
    // pin can read as "triggered" and silently block ALL motor output.
    HardwareLimitSwitchConfigs limitConfigs = new HardwareLimitSwitchConfigs();
    limitConfigs.ForwardLimitEnable = false;
    limitConfigs.ReverseLimitEnable = false;
    config.HardwareLimitSwitch = limitConfigs;

    // Apply config with retries
    boolean configOK = false;
    for (int i = 0; i < 3; i++) {
      StatusCode status = m_motor.getConfigurator().apply(config);
      System.out.println("[INTAKE] Config apply attempt " + (i + 1) + ": " + status);
      if (status.isOK()) {
        configOK = true;
        break;
      }
    }
    if (!configOK) {
      System.out.println("[INTAKE] *** WARNING: CONFIG APPLY FAILED AFTER 3 ATTEMPTS ***");
      System.out.println("[INTAKE] *** Motor may need firmware update in Tuner X! ***");
    }

    // Clear any sticky faults from previous brownouts
    m_motor.clearStickyFaults();
    System.out.println("[INTAKE] Sticky faults cleared");

    // Log firmware version safely
    try {
      int fwVersion = m_motor.getVersion().getValue();
      System.out.println("[INTAKE] Firmware version: " + fwVersion);
      if (fwVersion == 0) {
        System.out.println("[INTAKE] *** WARNING: FW version 0 — motor may need firmware flash in Tuner X! ***");
      }
    } catch (Exception e) {
      System.out.println("[INTAKE] *** Could not read firmware version: " + e.getMessage() + " ***");
    }
  }

  // ========== COMMANDS CALL THESE ==========

  /**
   * Run intake to grab game pieces.
   * Spins the roller to suck in.
   */
  public void runIntake() {
    double power = m_entryPowerPercent.getDouble(MotorConstants.kIntakePowerPercent) / 100.0;
    power = Math.max(0.0, Math.min(1.0, power));

    // Reduce power if battery is low
    if (m_powerManagement != null) {
      power *= m_powerManagement.getIntakePowerScale();
    }

    m_currentPower = power;
    double voltage = power * kNominalVoltage; // convert percent → voltage (e.g. 0.9 → 10.8V)
    StatusCode status = m_motor.setControl(m_voltageOut.withOutput(voltage));
    m_isRunning = true;
    m_entryStatus.setString("INTAKE @ " + (int)(power * 100) + "% (" + String.format("%.1f", voltage) + "V)");
    m_entrySetControl.setString(status.toString());

    // Log once per second so you can see in Driver Station console
    double now = Timer.getFPGATimestamp();
    if (now - m_lastLogTime > 1.0) {
      m_lastLogTime = now;
      System.out.println("[INTAKE] runIntake() voltage=" + String.format("%.1f", voltage)
          + "V setControl=" + status + " alive=" + m_motor.isAlive());
    }
  }

  /**
   * Run intake backwards to eject game pieces.
   */
  public void runReverse() {
    double power = -m_entryReversePowerPercent.getDouble(MotorConstants.kIntakeReversePowerPercent) / 100.0;
    power = Math.max(-1.0, Math.min(0.0, power));

    if (m_powerManagement != null) {
      power *= m_powerManagement.getIntakePowerScale();
    }

    m_currentPower = power;
    double voltage = power * kNominalVoltage; // negative voltage = reverse
    m_motor.setControl(m_voltageOut.withOutput(voltage));
    m_isRunning = true;
    m_entryStatus.setString("EJECTING @ " + (int)(Math.abs(power) * 100) + "%");
  }

  /**
   * Run at a specific power level.
   * Positive = intake, negative = eject.
   */
  public void runAtPower(double power) {
    m_currentPower = Math.max(-1.0, Math.min(1.0, power));
    m_motor.setControl(m_voltageOut.withOutput(m_currentPower * kNominalVoltage));
    m_isRunning = Math.abs(power) > 0.01;
  }

  /**
   * Stop the intake roller.
   */
  public void stop() {
    m_motor.setControl(m_voltageOut.withOutput(0));
    m_isRunning = false;
    m_currentPower = 0.0;
    m_entryStatus.setString("STOPPED");
  }

  // ========== INFO GETTERS ==========

  public boolean isRunning() { return m_isRunning; }

  public double getCurrentAmps() {
    return m_motor.getSupplyCurrent().getValueAsDouble();
  }

  public double getRPM() {
    return m_motor.getVelocity().getValueAsDouble() * 60.0;
  }

  public double getMotorTemp() {
    return m_motor.getDeviceTemp().getValueAsDouble();
  }

  public double getTotalRotations() { return m_totalRotations; }
  public double getPeakCurrent() { return m_peakCurrent; }

  // ========== PERIODIC - RUNS EVERY 20MS ==========

  @Override
  public void periodic() {
    // Use try-catch: if any CAN read returns null, we don't want periodic()
    // to throw and break the command scheduler for this subsystem
    try {
      double amps = getCurrentAmps();
      double rpm = getRPM();

      // Track stats
      m_totalRotations += Math.abs(m_motor.getVelocity().getValueAsDouble()) * 0.02;
      if (amps > m_peakCurrent) m_peakCurrent = amps;

      // Update Shuffleboard telemetry
      m_entryRPM.setDouble(rpm);
      m_entryAmps.setDouble(amps);
      m_entryTempC.setDouble(getMotorTemp());
      m_entryAppliedPct.setDouble(m_currentPower * 100);
      m_entryVoltage.setDouble(m_motor.getSupplyVoltage().getValueAsDouble());
      m_entryRunning.setBoolean(m_isRunning);
      m_entryAlive.setBoolean(m_motor.isAlive());
    } catch (Exception e) {
      // Don't let periodic() crash — it would break command scheduling
      m_entryStatus.setString("PERIODIC ERROR: " + e.getMessage());
    }

    // Auto-save tuning inputs to Preferences — throttled to every 5s to avoid loop overruns
    double now = Timer.getFPGATimestamp();
    if (now - m_lastPreferencesSaveTime > 5.0) {
      m_lastPreferencesSaveTime = now;
      Preferences.setDouble("Intake Power %",         m_entryPowerPercent.getDouble(MotorConstants.kIntakePowerPercent));
      Preferences.setDouble("Intake Reverse Power %", m_entryReversePowerPercent.getDouble(MotorConstants.kIntakeReversePowerPercent));
    }
  }
}
