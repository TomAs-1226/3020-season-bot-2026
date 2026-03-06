// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
 *                    TEAM 3020 - SHOOTER / FLYWHEEL SUBSYSTEM
 * ============================================================================
 *
 * Two Kraken X60 FOC motors on CAN 30 and CAN 31.
 * Both are independently velocity-controlled (no Follower mode).
 * Both configured CounterClockwise_Positive. Motors face AWAY from each other,
 * so Motor 2 receives NEGATED velocity to spin the correct physical direction.
 * Result: one green LED, one red LED — wheels pull game piece through.
 *
 * Uses VelocityVoltage (FOC) — closed-loop velocity control via Slot 0 gains.
 *
 * TUNING (Shuffleboard "Shooter" tab — no redeployment needed):
 *   "Target RPM"  — target speed when L1 is held
 *   "Idle RPM"    — speed held at all times between shots
 *   "Shooter kP"  — V per RPS of error  (catch-up aggressiveness)
 *   "Shooter kV"  — V per RPS of target (feedforward, steady-state)
 *   "Shooter kS"  — V to overcome static friction
 *   "Shooter kA"  — V per (RPS/s) during acceleration (start at 0)
 *
 * All values auto-save to Preferences and survive code deploys.
 *
 * BUTTON: L1 → ShootV1Command (spins up then auto-feeds when at speed)
 *         R1 → Reverse (clear jams)
 *
 * @author Baichen Yu
 * ============================================================================
 */
public class MotorGroup2Subsystem extends SubsystemBase {

  private final TalonFX m_motor1;  // CAN 30 — CounterClockwise_Positive
  private final TalonFX m_motor2;  // CAN 31 — Clockwise_Positive (inverted)

  // Each motor gets its own VelocityVoltage control object
  private final VelocityVoltage m_velocityControl1 =
      new VelocityVoltage(0).withSlot(0).withEnableFOC(true);
  private final VelocityVoltage m_velocityControl2 =
      new VelocityVoltage(0).withSlot(0).withEnableFOC(true);

  private final NeutralOut m_neutralControl = new NeutralOut();

  private double  m_targetRPM   = MotorConstants.kMotorGroup2TargetRPM;
  private boolean m_isReversing = false;

  // m_activeRPS: the velocity we WANT both motors at (0 = stopped, positive = running).
  // Re-applied every periodic() so CAN resets don't permanently kill the flywheel.
  private double m_activeRPS = 0.0;

  private double m_peakRPM     = 0.0;
  private int    m_spinupCount = 0;

  // Throttle Preferences saves to every 5s to prevent loop overrun
  private double m_lastPreferencesSaveTime = 0.0;

  // ===== SHUFFLEBOARD — "Shooter" tab =====
  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Shooter");

  // ----- Tunable inputs (editable in Shuffleboard) -----
  private final GenericEntry m_entryTargetRPM =
      m_tab.add("Target RPM",  MotorConstants.kMotorGroup2TargetRPM).getEntry();
  private final GenericEntry m_entryIdleRPM =
      m_tab.add("Idle RPM",    MotorConstants.kShooterIdleRPM).getEntry();
  private final GenericEntry m_entryKp =
      m_tab.add("Shooter kP",  MotorConstants.kShooterKp).getEntry();
  private final GenericEntry m_entryKv =
      m_tab.add("Shooter kV",  MotorConstants.kShooterKv).getEntry();
  private final GenericEntry m_entryKs =
      m_tab.add("Shooter kS",  MotorConstants.kShooterKs).getEntry();
  private final GenericEntry m_entryKa =
      m_tab.add("Shooter kA",  MotorConstants.kShooterKa).getEntry();

  // ----- Status / telemetry (read-only display) -----
  private final GenericEntry m_entryAvgRPM      = m_tab.add("Avg RPM",        0.0).getEntry();
  private final GenericEntry m_entrySetpoint    = m_tab.add("RPM Setpoint",   0.0).getEntry();
  private final GenericEntry m_entryRPMError    = m_tab.add("RPM Error",      0.0).getEntry();
  private final GenericEntry m_entryTotalAmps   = m_tab.add("Total Amps",     0.0).getEntry();
  private final GenericEntry m_entryPeakRPM     = m_tab.add("Peak RPM",       0.0).getEntry();
  private final GenericEntry m_entrySpinupCount = m_tab.add("Spinup Count",     0).getEntry();
  private final GenericEntry m_entryStatus      = m_tab.add("Shooter Status", "INIT").getEntry();
  private final GenericEntry m_entryOnTarget    = m_tab.add("On Target",      false).getEntry();
  private final GenericEntry m_entryReversing   = m_tab.add("Reversing",      false).getEntry();
  private final GenericEntry m_entryMotor1Alive   = m_tab.add("Motor1 Alive",   false).getEntry();
  private final GenericEntry m_entryMotor2Alive   = m_tab.add("Motor2 Alive",   false).getEntry();

  public MotorGroup2Subsystem() {
    System.out.println("[SHOOTER] Starting up (VelocityVoltage FOC, dual independent control)...");

    m_motor1 = new TalonFX(MotorConstants.kMotorGroup2Motor1ID);
    m_motor2 = new TalonFX(MotorConstants.kMotorGroup2Motor2ID);

    configureMotor1();
    configureMotor2();

    // Load saved tuning values from Preferences → push to Shuffleboard entries
    m_entryTargetRPM.setDouble(Preferences.getDouble("Shooter Target RPM", MotorConstants.kMotorGroup2TargetRPM));
    m_entryIdleRPM.setDouble(  Preferences.getDouble("Shooter Idle RPM",   MotorConstants.kShooterIdleRPM));
    m_entryKp.setDouble(       Preferences.getDouble("Shooter kP",         MotorConstants.kShooterKp));
    m_entryKv.setDouble(       Preferences.getDouble("Shooter kV",         MotorConstants.kShooterKv));
    m_entryKs.setDouble(       Preferences.getDouble("Shooter kS",         MotorConstants.kShooterKs));
    m_entryKa.setDouble(       Preferences.getDouble("Shooter kA",         MotorConstants.kShooterKa));

    // Spin up to idle immediately on boot if idle mode is enabled
    if (MotorConstants.kUseShooterIdle) {
      double idleRPS = m_entryIdleRPM.getDouble(MotorConstants.kShooterIdleRPM) / 60.0;
      m_activeRPS = idleRPS;
      m_motor1.setControl(m_velocityControl1.withVelocity(-idleRPS));
      m_motor2.setControl(m_velocityControl2.withVelocity(idleRPS));
    }

    System.out.println("[SHOOTER] Motor1 CAN " + MotorConstants.kMotorGroup2Motor1ID
        + " (CCW+), Motor2 CAN " + MotorConstants.kMotorGroup2Motor2ID + " (CW+ inverted)");
  }

  /** Configure CAN 30 — CounterClockwise_Positive (default direction). */
  private void configureMotor1() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.StatorCurrentLimit       = 120;
    currentLimits.StatorCurrentLimitEnable = true;
    currentLimits.SupplyCurrentLimit       = MotorConstants.getMotorGroup2CurrentLimit();
    currentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits = currentLimits;

    MotorOutputConfigs motorOutput = new MotorOutputConfigs();
    motorOutput.Inverted    = InvertedValue.CounterClockwise_Positive;
    motorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput = motorOutput;

    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = MotorConstants.kShooterKp;
    slot0.kI = MotorConstants.kShooterKi;
    slot0.kD = MotorConstants.kShooterKd;
    slot0.kV = MotorConstants.kShooterKv;
    slot0.kS = MotorConstants.kShooterKs;
    slot0.kA = MotorConstants.kShooterKa;
    config.Slot0 = slot0;

    for (int i = 0; i < 3; i++) {
      if (m_motor1.getConfigurator().apply(config).isOK()) break;
    }
  }

  /** Configure CAN 31 — Clockwise_Positive (INVERTED vs motor 1).
   *  Same PID gains, same current limits. The inversion makes it spin
   *  the opposite physical direction when given the same positive velocity. */
  private void configureMotor2() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.StatorCurrentLimit       = 120;
    currentLimits.StatorCurrentLimitEnable = true;
    currentLimits.SupplyCurrentLimit       = MotorConstants.getMotorGroup2CurrentLimit();
    currentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits = currentLimits;

    MotorOutputConfigs motorOutput = new MotorOutputConfigs();
    motorOutput.Inverted    = InvertedValue.CounterClockwise_Positive;
    motorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput = motorOutput;

    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = MotorConstants.kShooterKp;
    slot0.kI = MotorConstants.kShooterKi;
    slot0.kD = MotorConstants.kShooterKd;
    slot0.kV = MotorConstants.kShooterKv;
    slot0.kS = MotorConstants.kShooterKs;
    slot0.kA = MotorConstants.kShooterKa;
    config.Slot0 = slot0;

    for (int i = 0; i < 3; i++) {
      if (m_motor2.getConfigurator().apply(config).isOK()) break;
    }
  }

  // ========== PUBLIC API ==========

  /** Spin up to target RPM (reads from Shuffleboard — user edits in "Shooter" tab). */
  public void runForward() {
    m_targetRPM = m_entryTargetRPM.getDouble(MotorConstants.kMotorGroup2TargetRPM);
    double newRPS = m_targetRPM / 60.0;
    // Only count a new spinup when transitioning from idle/stopped to target speed
    if (m_activeRPS != newRPS) {
      m_spinupCount++;
    }
    m_activeRPS = newRPS;
    m_motor1.setControl(m_velocityControl1.withVelocity(-m_activeRPS));
    m_motor2.setControl(m_velocityControl2.withVelocity(m_activeRPS));
    m_isReversing = false;
  }

  /** Reverse shooter to clear jams. */
  public void runReverse() {
    m_targetRPM = m_entryTargetRPM.getDouble(MotorConstants.kMotorGroup2TargetRPM);
    m_activeRPS = -(m_targetRPM / 60.0);
    m_motor1.setControl(m_velocityControl1.withVelocity(-m_activeRPS));
    m_motor2.setControl(m_velocityControl2.withVelocity(m_activeRPS));
    m_isReversing = true;
  }

  /** Run at an explicit RPM (used by RunMotorGroup2ReverseCommand for slow unjam). */
  public void runAtRPM(double rpm) {
    m_activeRPS = rpm / 60.0;
    m_motor1.setControl(m_velocityControl1.withVelocity(-m_activeRPS));
    m_motor2.setControl(m_velocityControl2.withVelocity(m_activeRPS));
    m_isReversing = rpm < 0;
  }

  /**
   * True when average RPM is within tolerance of the target.
   * Gates the feeder — ShootV1Command pattern.
   */
  public boolean isOnTarget() {
    return Math.abs(getAverageRPM() - m_targetRPM) < MotorConstants.kShooterOnTargetToleranceRPM;
  }

  /**
   * Stop shooting. Returns to idle if enabled, otherwise full stop.
   * Directly applies the idle velocity (no coast-to-idle delay) so
   * the shooter reliably holds idle through feeder voltage sags.
   */
  public void stopMotors() {
    m_isReversing = false;
    if (MotorConstants.kUseShooterIdle) {
      double idleRPS = m_entryIdleRPM.getDouble(MotorConstants.kShooterIdleRPM) / 60.0;
      m_activeRPS = idleRPS;
      m_motor1.setControl(m_velocityControl1.withVelocity(-idleRPS));
      m_motor2.setControl(m_velocityControl2.withVelocity(idleRPS));
      m_entryStatus.setString("IDLE @ " + (int)(idleRPS * 60) + " RPM");
    } else {
      m_activeRPS = 0.0;
      m_motor1.setControl(m_neutralControl);
      m_motor2.setControl(m_neutralControl);
      m_entryStatus.setString("STOPPED");
    }
  }

  /** Force a complete stop. */
  public void forceStop() {
    m_activeRPS = 0.0;
    m_motor1.setControl(m_neutralControl);
    m_motor2.setControl(m_neutralControl);
    m_entryStatus.setString("FORCE STOPPED");
  }

  /**
   * Update the "Shooter Status" field in the Shuffleboard "Shooter" tab.
   * Called by ShootV1Command to show spinup progress.
   */
  public void setShooterStatus(String status) {
    m_entryStatus.setString(status);
  }

  // ========== GETTERS ==========

  public double getAverageRPM() {
    // Both motors report positive velocity when spinning their "positive" direction.
    // Motor2 is inverted in config so its positive = opposite physical direction of Motor1.
    // Taking absolute values ensures we're averaging actual speeds, not cancelling out.
    double rpm1 = Math.abs(m_motor1.getVelocity().getValueAsDouble() * 60.0);
    double rpm2 = Math.abs(m_motor2.getVelocity().getValueAsDouble() * 60.0);
    return (rpm1 + rpm2) / 2.0;
  }

  public double getTotalCurrentAmps() {
    return m_motor1.getSupplyCurrent().getValueAsDouble()
         + m_motor2.getSupplyCurrent().getValueAsDouble();
  }

  public boolean isReversing()  { return m_isReversing; }
  public double  getTargetRPM() { return m_targetRPM;   }

  // ========== PERIODIC - RUNS EVERY 20MS ==========

  @Override
  public void periodic() {
    // Re-apply the active velocity setpoint to BOTH motors EVERY loop.
    // This recovers from CAN resets caused by voltage brownouts.
    if (m_activeRPS != 0.0) {
      m_motor1.setControl(m_velocityControl1.withVelocity(-m_activeRPS));
      m_motor2.setControl(m_velocityControl2.withVelocity(m_activeRPS));
    }

    double now = Timer.getFPGATimestamp();

    double avgRPM    = getAverageRPM();
    double totalAmps = getTotalCurrentAmps();

    if (avgRPM > m_peakRPM) m_peakRPM = avgRPM;

    // Telemetry → Shuffleboard "Shooter" tab
    m_entryAvgRPM.setDouble(avgRPM);
    m_entrySetpoint.setDouble(m_activeRPS * 60.0);
    m_entryRPMError.setDouble(m_targetRPM - avgRPM);
    m_entryTotalAmps.setDouble(totalAmps);
    m_entryPeakRPM.setDouble(m_peakRPM);
    m_entrySpinupCount.setInteger(m_spinupCount);
    m_entryOnTarget.setBoolean(isOnTarget());
    m_entryReversing.setBoolean(m_isReversing);
    m_entryMotor1Alive.setBoolean(m_motor1.isAlive());
    m_entryMotor2Alive.setBoolean(m_motor2.isAlive());

    // Auto-save tuning inputs to Preferences — throttled to every 5s to prevent loop overrun
    if (now - m_lastPreferencesSaveTime > 5.0) {
      m_lastPreferencesSaveTime = now;
      Preferences.setDouble("Shooter Target RPM", m_entryTargetRPM.getDouble(MotorConstants.kMotorGroup2TargetRPM));
      Preferences.setDouble("Shooter Idle RPM",   m_entryIdleRPM.getDouble(MotorConstants.kShooterIdleRPM));
      Preferences.setDouble("Shooter kP",         m_entryKp.getDouble(MotorConstants.kShooterKp));
      Preferences.setDouble("Shooter kV",         m_entryKv.getDouble(MotorConstants.kShooterKv));
      Preferences.setDouble("Shooter kS",         m_entryKs.getDouble(MotorConstants.kShooterKs));
      Preferences.setDouble("Shooter kA",         m_entryKa.getDouble(MotorConstants.kShooterKa));
    }
  }
}
