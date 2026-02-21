// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

/**
 * ============================================================================
 *                    TEAM 3020 - SHOOTER SUBSYSTEM
 * ============================================================================
 *
 * This is the shooter! It's got 4 Kraken X60 motors all spinning together.
 * One motor is the "leader" and the other 3 just copy what it does.
 *
 * HOW IT WORKS:
 *   - Press L1 and the shooter spins up to the target RPM
 *   - Let go and it stays spinning at idle speed (500 RPM) to save spinup time
 *   - The idle feature means your next shot spins up faster!
 *
 * MOTORS:
 *   - Motor 1 (CAN 3) = Leader - this one gets the commands
 *   - Motor 2 (CAN 4) = Follower
 *   - Motor 3 (CAN 6) = Follower
 *   - Motor 4 (CAN 7) = Follower
 *
 * TUNING:
 *   You can change the shooter speed on SmartDashboard:
 *   Look for "Shooter Target RPM" and adjust it up or down.
 *
 * @author Baichen Yu
 * ============================================================================
 */
public class MotorGroup2Subsystem extends SubsystemBase {

  // The 4 shooter motors
  private final TalonFX m_motor1; // Leader - this one gets the commands
  private final TalonFX m_motor2; // Follower - just copies motor 1
  private final TalonFX m_motor3; // Follower
  private final TalonFX m_motor4; // Follower

  // Velocity control - this is how we spin at a specific RPM
  private final VelocityVoltage m_velocityControl = new VelocityVoltage(0).withSlot(0);

  // What RPM are we trying to hit?
  private double m_targetRPM = MotorConstants.kMotorGroup2TargetRPM;

  // Idle mode - keeps the shooter spinning slowly between shots
  private boolean m_idleActive = false;

  // Stats for the dashboard
  private double m_peakRPM = 0.0;      // Highest RPM we've hit
  private double m_peakCurrent = 0.0;  // Highest amps we've drawn
  private int m_spinupCount = 0;       // How many times we've spun up
  private boolean m_isReversing = false; // Are we running backwards?

  /**
   * Creates the shooter subsystem.
   * Sets up all 4 motors and configures them to work together.
   */
  public MotorGroup2Subsystem() {
    System.out.println("[SHOOTER] Starting up...");

    // Connect to all 4 motors on the CAN bus
    CANBus canBus = new CANBus(MotorConstants.kMotorGroup2CANBus);
    m_motor1 = new TalonFX(MotorConstants.kMotorGroup2Motor1ID, canBus);
    m_motor2 = new TalonFX(MotorConstants.kMotorGroup2Motor2ID, canBus);
    m_motor3 = new TalonFX(MotorConstants.kMotorGroup2Motor3ID, canBus);
    m_motor4 = new TalonFX(MotorConstants.kMotorGroup2Motor4ID, canBus);

    // Configure all motors the same way
    configureTalonFX(m_motor1, InvertedValue.CounterClockwise_Positive);
    configureTalonFX(m_motor2, InvertedValue.CounterClockwise_Positive);
    configureTalonFX(m_motor3, InvertedValue.CounterClockwise_Positive);
    configureTalonFX(m_motor4, InvertedValue.CounterClockwise_Positive);

    // Make motors 2, 3, 4 follow motor 1
    // This way we only send commands to motor 1 and the others copy it
    m_motor2.setControl(new Follower(MotorConstants.kMotorGroup2Motor1ID, MotorAlignmentValue.Aligned));
    m_motor3.setControl(new Follower(MotorConstants.kMotorGroup2Motor1ID, MotorAlignmentValue.Aligned));
    m_motor4.setControl(new Follower(MotorConstants.kMotorGroup2Motor1ID, MotorAlignmentValue.Aligned));

    // Put the target RPM on the dashboard so drivers can adjust it
    SmartDashboard.putNumber("Shooter Target RPM", m_targetRPM);

    System.out.println("[SHOOTER] 4 motors ready!");
    System.out.println("[SHOOTER] Idle speed: " + MotorConstants.kShooterIdleRPM + " RPM");
  }

  /**
   * Configures a single motor with current limits and PID settings.
   * We call this for each of the 4 motors.
   */
  private void configureTalonFX(TalonFX motor, InvertedValue inverted) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Current limits - don't let the motors draw too much power
    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.SupplyCurrentLimit = MotorConstants.getMotorGroup2CurrentLimit();
    currentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits = currentLimits;

    // Motor output settings
    MotorOutputConfigs motorOutput = new MotorOutputConfigs();
    motorOutput.Inverted = inverted;
    motorOutput.NeutralMode = NeutralModeValue.Coast; // Coast when not powered
    config.MotorOutput = motorOutput;

    // PID settings for velocity control
    // These numbers control how smoothly the motor reaches target speed
    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = MotorConstants.getMotorGroup2Kp();
    slot0.kV = MotorConstants.getMotorGroup2Kv();
    slot0.kS = MotorConstants.getMotorGroup2Ks();
    slot0.kA = MotorConstants.getMotorGroup2Ka();
    config.Slot0 = slot0;

    // Ramp rate - how fast the motor speeds up
    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = MotorConstants.kRampRate;

    // Try to apply config up to 3 times (sometimes CAN is flaky)
    for (int i = 0; i < 3; i++) {
      if (motor.getConfigurator().apply(config).isOK()) break;
    }
  }

  // ========== COMMANDS CALL THESE ==========

  /**
   * Spin up the shooter forward!
   * Uses whatever RPM is set on the dashboard.
   */
  public void runForward() {
    m_targetRPM = SmartDashboard.getNumber("Shooter Target RPM", MotorConstants.kMotorGroup2TargetRPM);
    setShooterRPM(m_targetRPM);
  }

  /**
   * Spin the shooter backwards.
   * Useful if a game piece gets stuck.
   */
  public void runReverse() {
    m_targetRPM = SmartDashboard.getNumber("Shooter Target RPM", MotorConstants.kMotorGroup2TargetRPM);
    setShooterRPM(-m_targetRPM);
  }

  /**
   * Run at a specific RPM.
   * Use this if you want to override the dashboard value.
   */
  public void runAtRPM(double rpm) {
    setShooterRPM(rpm);
  }

  /**
   * Actually sets the shooter speed.
   * This is what the other methods call.
   */
  private void setShooterRPM(double rpm) {
    // Track that we've started shooting (for idle mode)
    if (Math.abs(rpm) > 0 && !m_idleActive) {
      m_idleActive = true;
      m_spinupCount++;
    }

    // Track if we're going backwards
    m_isReversing = rpm < 0;
    m_targetRPM = Math.abs(rpm);

    // Send the command to motor 1 (others will follow)
    // Divide by 60 because Phoenix 6 uses rotations per second, not RPM
    m_motor1.setControl(m_velocityControl.withVelocity(rpm / 60.0));
  }

  /**
   * Stop the shooter.
   * If idle is enabled, it'll drop to idle speed instead of stopping completely.
   */
  public void stopMotors() {
    if (m_idleActive && MotorConstants.kUseShooterIdle) {
      // Don't fully stop - just idle
      m_motor1.setControl(m_velocityControl.withVelocity(MotorConstants.kShooterIdleRPM / 60.0));
      SmartDashboard.putString("Shooter Status", "IDLE");
    } else {
      // Full stop
      m_motor1.setControl(m_velocityControl.withVelocity(0));
      SmartDashboard.putString("Shooter Status", "STOPPED");
    }
  }

  /**
   * Force a complete stop - ignores idle mode.
   * Use this when you really want it to stop.
   */
  public void forceStop() {
    m_motor1.setControl(m_velocityControl.withVelocity(0));
    m_idleActive = false;
    SmartDashboard.putString("Shooter Status", "FORCE STOPPED");
  }

  // ========== INFO GETTERS ==========

  /** Is the shooter running backwards? */
  public boolean isReversing() {
    return m_isReversing;
  }

  /** Is the shooter in idle mode (spinning slowly)? */
  public boolean isIdling() {
    return m_idleActive && getAverageRPM() < MotorConstants.kShooterIdleRPM + 100;
  }

  /** Average RPM of all 4 motors */
  public double getAverageRPM() {
    double total = m_motor1.getVelocity().getValueAsDouble() +
                   m_motor2.getVelocity().getValueAsDouble() +
                   m_motor3.getVelocity().getValueAsDouble() +
                   m_motor4.getVelocity().getValueAsDouble();
    return total * 15.0; // *60/4 (convert RPS to RPM and average)
  }

  /** Total amps being drawn by all 4 motors */
  public double getTotalCurrentAmps() {
    return m_motor1.getSupplyCurrent().getValueAsDouble() +
           m_motor2.getSupplyCurrent().getValueAsDouble() +
           m_motor3.getSupplyCurrent().getValueAsDouble() +
           m_motor4.getSupplyCurrent().getValueAsDouble();
  }

  /** RPM of just the leader motor */
  public double getLeaderRPM() {
    return m_motor1.getVelocity().getValueAsDouble() * 60.0;
  }

  // ========== PERIODIC - RUNS EVERY 20MS ==========

  @Override
  public void periodic() {
    double avgRPM = getAverageRPM();
    double totalAmps = getTotalCurrentAmps();

    // Track peak values
    if (avgRPM > m_peakRPM) m_peakRPM = avgRPM;
    if (totalAmps > m_peakCurrent) m_peakCurrent = totalAmps;

    // Put everything on SmartDashboard
    SmartDashboard.putNumber("Shooter Avg RPM", avgRPM);
    SmartDashboard.putNumber("Shooter Target RPM", m_targetRPM);
    SmartDashboard.putNumber("Shooter Total Amps", totalAmps);
    SmartDashboard.putNumber("Shooter RPM Error", m_targetRPM - avgRPM);

    // Peak stats
    SmartDashboard.putNumber("Shooter Peak RPM", m_peakRPM);
    SmartDashboard.putNumber("Shooter Peak Amps", m_peakCurrent);
    SmartDashboard.putNumber("Shooter Spinup Count", m_spinupCount);

    // Status flags
    SmartDashboard.putBoolean("Shooter Reversing", m_isReversing);
    SmartDashboard.putBoolean("Shooter Idle Active", m_idleActive);

    // Are all motors working?
    boolean allAlive = m_motor1.isAlive() && m_motor2.isAlive() &&
                       m_motor3.isAlive() && m_motor4.isAlive();
    SmartDashboard.putBoolean("Shooter All Alive", allAlive);
  }
}
