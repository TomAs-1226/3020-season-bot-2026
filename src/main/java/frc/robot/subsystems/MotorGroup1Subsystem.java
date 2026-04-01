// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
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
 *                    TEAM 3020 - FEEDER SUBSYSTEM
 * ============================================================================
 *
 * Controls the two motors that push game pieces from the intake into the shooter.
 *
 * MOTORS:
 *   Motor 1 (CAN 40) — Hopper        — Kraken X60 FOC, INVERTED
 *   Motor 2 (CAN 41) — Feed Roller   — Kraken X60 FOC
 *
 * TUNING (Shuffleboard "Feeder" tab — no redeployment needed):
 *   "Hopper Power %"      — duty cycle for the hopper only (default 20%). Run slower.
 *   "Feed Roller Power %" — duty cycle for the feed roller (default 35%). Can run faster.
 *   Values auto-save to Preferences and survive code deploys.
 *
 * BUTTON: Cross (X) = manual feeder override (hold)
 *         L1        = auto-feed via ShootV1Command when shooter is on target
 *
 * @author Baichen Yu
 * ============================================================================
 */
public class MotorGroup1Subsystem extends SubsystemBase {

  // The two feeder motors
  private final TalonFX m_motor1; // CAN 40 — Hopper
  private final TalonFX m_motor2; // CAN 41 — Feed Roller

  // Duty cycle control — FOC gives better torque at low speeds
  private final DutyCycleOut m_motor1DutyCycle = new DutyCycleOut(0).withEnableFOC(true);
  private final DutyCycleOut m_motor2DutyCycle = new DutyCycleOut(0).withEnableFOC(true);

  // Coast-out — used by stopMotors() for gradual natural spindown (no jolt to game pieces)
  private final CoastOut m_coastOut = new CoastOut();

  // Current commanded powers (0.0–1.0); updated from Shuffleboard each call
  private double m_hopperPower = MotorConstants.kMotorGroup1HopperPowerPercent / 100.0;
  private double m_rollerPower = MotorConstants.kMotorGroup1PowerPercent / 100.0;

  // Throttle Preferences saves to every 5s to prevent loop overrun
  private double m_lastPreferencesSaveTime = 0.0;

  // ===== SHUFFLEBOARD — "Feeder" tab =====
  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Feeder");

  // ----- Tunable inputs (editable in Shuffleboard) -----
  /**
   * Hopper duty cycle in percent (0–100). Runs slower than the feed roller.
   * Default: 20% — increase gradually until game pieces advance reliably.
   */
  private final GenericEntry m_entryHopperPowerPercent =
      m_tab.add("Hopper Power %", MotorConstants.kMotorGroup1HopperPowerPercent).getEntry();

  /**
   * Feed Roller duty cycle in percent (0–100).
   * Default: 35% — increase until game pieces feed reliably without causing
   * noticeable shooter RPM drop.  Keep below 60% to avoid voltage sag.
   */
  private final GenericEntry m_entryRollerPowerPercent =
      m_tab.add("Feed Roller Power %", MotorConstants.kMotorGroup1PowerPercent).getEntry();

  // ----- Status / telemetry -----
  private final GenericEntry m_entryHopperRPM       = m_tab.add("Hopper RPM",           0.0).getEntry();
  private final GenericEntry m_entryRollerRPM        = m_tab.add("Feed Roller RPM",      0.0).getEntry();
  private final GenericEntry m_entryHopperAmps       = m_tab.add("Hopper Amps",          0.0).getEntry();
  private final GenericEntry m_entryRollerAmps       = m_tab.add("Feed Roller Amps",     0.0).getEntry();
  private final GenericEntry m_entryRunning          = m_tab.add("Running",              false).getEntry();
  private final GenericEntry m_entryHopperApplied    = m_tab.add("Hopper Applied %",     0.0).getEntry();
  private final GenericEntry m_entryRollerApplied    = m_tab.add("Roller Applied %",     0.0).getEntry();
  private final GenericEntry m_entryHopperAlive      = m_tab.add("Hopper Alive",         false).getEntry();
  private final GenericEntry m_entryRollerAlive      = m_tab.add("Feed Roller Alive",    false).getEntry();

  /**
   * Creates the feeder subsystem.
   */
  public MotorGroup1Subsystem() {
    System.out.println("[FEEDER] Starting up...");

    m_motor1 = new TalonFX(MotorConstants.kMotorGroup1Motor1ID); // Hopper
    m_motor2 = new TalonFX(MotorConstants.kMotorGroup1Motor2ID); // Feed roller

    // Hopper (motor1) is inverted — CW = positive = outtake direction for the hopper geometry
    configureTalonFX(m_motor1, InvertedValue.Clockwise_Positive);
    configureTalonFX(m_motor2, InvertedValue.CounterClockwise_Positive);

    // Load saved values from Preferences into Shuffleboard entries
    m_entryHopperPowerPercent.setDouble(
        Preferences.getDouble("Hopper Power %", MotorConstants.kMotorGroup1HopperPowerPercent));
    m_entryRollerPowerPercent.setDouble(
        Preferences.getDouble("Feed Roller Power %", MotorConstants.kMotorGroup1PowerPercent));

    System.out.println("[FEEDER] 2 motors ready! Hopper: "
        + MotorConstants.kMotorGroup1HopperPowerPercent + "%  Roller: "
        + MotorConstants.kMotorGroup1PowerPercent + "%  Current limit: "
        + MotorConstants.getMotorGroup1CurrentLimit() + "A");
  }

  /**
   * Configures a single motor with current limits and output settings.
   */
  private void configureTalonFX(TalonFX motor, InvertedValue inverted) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.SupplyCurrentLimit       = MotorConstants.getMotorGroup1CurrentLimit();
    currentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits = currentLimits;

    MotorOutputConfigs motorOutput = new MotorOutputConfigs();
    motorOutput.Inverted    = inverted;
    motorOutput.NeutralMode = MotorConstants.kUseBrakeMode
        ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.MotorOutput = motorOutput;

    // Ramp rate — smooth acceleration prevents sudden current spikes
    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = MotorConstants.kRampRate;

    for (int i = 0; i < 3; i++) {
      if (motor.getConfigurator().apply(config).isOK()) break;
    }
  }

  // ========== COMMANDS CALL THESE ==========

  /**
   * Run feeder motors: hopper at "Hopper Power %", feed roller at "Feed Roller Power %".
   * Values read every call so live Shuffleboard changes take effect immediately.
   */
  public void runMotors() {
    m_hopperPower = m_entryHopperPowerPercent.getDouble(MotorConstants.kMotorGroup1HopperPowerPercent) / 100.0;
    m_hopperPower = Math.max(0.0, Math.min(1.0, m_hopperPower));

    m_rollerPower = m_entryRollerPowerPercent.getDouble(MotorConstants.kMotorGroup1PowerPercent) / 100.0;
    m_rollerPower = Math.max(0.0, Math.min(1.0, m_rollerPower));

    m_motor1.setControl(m_motor1DutyCycle.withOutput(m_hopperPower));
    m_motor2.setControl(m_motor2DutyCycle.withOutput(m_rollerPower));
    m_entryRunning.setBoolean(true);
  }

  /** Run hopper only (CAN 40) — used for intake + hopper on Circle.
   *  Explicitly stops the feed roller (CAN 41) so the indexer never runs during intake. */
  public void runHopperOnly() {
    m_hopperPower = m_entryHopperPowerPercent.getDouble(MotorConstants.kMotorGroup1HopperPowerPercent) / 100.0;
    m_hopperPower = Math.max(0.0, Math.min(1.0, m_hopperPower));
    m_motor1.setControl(m_motor1DutyCycle.withOutput(m_hopperPower));
    m_motor2.setControl(m_coastOut); // keep feed roller stopped during intake
    m_entryRunning.setBoolean(true);
  }

  /** Run feeder in reverse — used for full outtake (L2). */
  public void runMotorsReverse() {
    m_hopperPower = m_entryHopperPowerPercent.getDouble(MotorConstants.kMotorGroup1HopperPowerPercent) / 100.0;
    m_hopperPower = Math.max(0.0, Math.min(1.0, m_hopperPower));

    m_rollerPower = m_entryRollerPowerPercent.getDouble(MotorConstants.kMotorGroup1PowerPercent) / 100.0;
    m_rollerPower = Math.max(0.0, Math.min(1.0, m_rollerPower));

    m_motor1.setControl(m_motor1DutyCycle.withOutput(-m_hopperPower));
    m_motor2.setControl(m_motor2DutyCycle.withOutput(-m_rollerPower));
    m_entryRunning.setBoolean(true);
  }

  /**
   * Stop the feeder motors — coast to a natural stop rather than snapping to zero.
   * Prevents jolting game pieces that are still in the feed path.
   */
  public void stopMotors() {
    m_motor1.setControl(m_coastOut);
    m_motor2.setControl(m_coastOut);
    m_entryRunning.setBoolean(false);
  }

  // ========== PERIODIC - RUNS EVERY 20MS ==========

  @Override
  public void periodic() {
    // Telemetry → Shuffleboard "Feeder" tab
    m_entryHopperRPM.setDouble(m_motor1.getVelocity().getValueAsDouble() * 60.0);
    m_entryRollerRPM.setDouble(m_motor2.getVelocity().getValueAsDouble() * 60.0);
    m_entryHopperAmps.setDouble(m_motor1.getSupplyCurrent().getValueAsDouble());
    m_entryRollerAmps.setDouble(m_motor2.getSupplyCurrent().getValueAsDouble());
    m_entryHopperApplied.setDouble(m_hopperPower * 100.0);
    m_entryRollerApplied.setDouble(m_rollerPower * 100.0);
    m_entryHopperAlive.setBoolean(m_motor1.isAlive());
    m_entryRollerAlive.setBoolean(m_motor2.isAlive());

    // Auto-save tuning inputs to Preferences — throttled to every 5s to prevent loop overrun
    double now = Timer.getFPGATimestamp();
    if (now - m_lastPreferencesSaveTime > 5.0) {
      m_lastPreferencesSaveTime = now;
      Preferences.setDouble("Hopper Power %",
          m_entryHopperPowerPercent.getDouble(MotorConstants.kMotorGroup1HopperPowerPercent));
      Preferences.setDouble("Feed Roller Power %",
          m_entryRollerPowerPercent.getDouble(MotorConstants.kMotorGroup1PowerPercent));
    }
  }
}
