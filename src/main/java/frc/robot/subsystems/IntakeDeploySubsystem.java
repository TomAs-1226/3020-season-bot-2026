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
 *                    TEAM 3020 - INTAKE ARM SUBSYSTEM
 * ============================================================================
 *
 * This controls the intake arm that swings down to the floor to grab game
 * pieces. It uses "stall detection" - when it hits the floor or the top,
 * the motor stalls and we automatically stop it.
 *
 * HOW IT WORKS:
 *   - D-pad Down = Swing arm down to the floor
 *   - D-pad Up   = Bring arm back up
 *   - Square     = Auto intake (arm + rollers together - use this one!)
 *
 * The arm will automatically stop when it hits the mechanical stop at
 * either end. No limit switches needed!
 *
 * MOTOR:
 *   - CAN ID 8 = Kraken X60
 *
 * STALL DETECTION:
 *   When the arm hits a stop, the motor current spikes and the velocity
 *   drops. We detect this and stop the motor to protect it.
 *   - Current threshold: 35A
 *   - Velocity threshold: 50 RPM
 *   - Time threshold: 0.1 seconds
 *
 * @author Baichen Yu
 * ============================================================================
 */
public class IntakeDeploySubsystem extends SubsystemBase {

  /**
   * The different states the arm can be in.
   */
  public enum DeployState {
    UP,           // Arm is all the way up
    DOWN,         // Arm is all the way down
    MOVING_UP,    // Arm is going up
    MOVING_DOWN,  // Arm is going down
    UNKNOWN       // We don't know where it is (startup or after force stop)
  }

  // The arm motor
  private final TalonFX m_motor;
  private final DutyCycleOut m_dutyCycle = new DutyCycleOut(0).withEnableFOC(true);

  // Current state
  private DeployState m_state = DeployState.UP;  // Assume we start up
  private boolean m_isMoving = false;

  // Stall detection
  private boolean m_stallDetected = false;
  private double m_stallStartTime = 0.0;
  private boolean m_highCurrentDetected = false;

  /**
   * Creates the intake deploy subsystem.
   */
  public IntakeDeploySubsystem() {
    System.out.println("[ARM] Starting up...");

    // Connect to the motor
    m_motor = new TalonFX(MotorConstants.kMotorGroup4MotorID);
    configureMotor();

    // Reset position - assume we're at the top
    m_motor.setPosition(0);

    System.out.println("[ARM] Intake arm ready - CAN ID " + MotorConstants.kMotorGroup4MotorID);
    System.out.println("[ARM] Down power: " + MotorConstants.kDeployDownPowerPercent + "%");
    System.out.println("[ARM] Up power: " + MotorConstants.kDeployUpPowerPercent + "%");
  }

  /**
   * Configures the motor with current limits and output settings.
   */
  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Current limiting
    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.SupplyCurrentLimit = MotorConstants.kKrakenCurrentLimit;
    currentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits = currentLimits;

    // Motor output settings
    MotorOutputConfigs motorOutput = new MotorOutputConfigs();
    motorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorOutput.NeutralMode = NeutralModeValue.Brake;  // Hold position when stopped
    config.MotorOutput = motorOutput;

    // Smooth ramp to prevent jerky movement
    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.15;

    // Apply config
    for (int i = 0; i < 3; i++) {
      if (m_motor.getConfigurator().apply(config).isOK()) break;
    }
  }

  // ========== COMMANDS CALL THESE ==========

  /**
   * Swing the intake arm down to the floor.
   * Will automatically stop when it hits the floor.
   */
  public void deployDown() {
    // Don't do anything if we're already down
    if (m_state == DeployState.DOWN) return;

    m_state = DeployState.MOVING_DOWN;
    m_isMoving = true;
    m_stallDetected = false;
    m_highCurrentDetected = false;

    m_motor.setControl(m_dutyCycle.withOutput(MotorConstants.kDeployDownPowerPercent / 100.0));
    SmartDashboard.putString("Arm Status", "GOING DOWN");
  }

  /**
   * Bring the intake arm back up.
   * Will automatically stop when it hits the top.
   */
  public void deployUp() {
    // Don't do anything if we're already up
    if (m_state == DeployState.UP) return;

    m_state = DeployState.MOVING_UP;
    m_isMoving = true;
    m_stallDetected = false;
    m_highCurrentDetected = false;

    // Negative power to go up
    m_motor.setControl(m_dutyCycle.withOutput(-MotorConstants.kDeployUpPowerPercent / 100.0));
    SmartDashboard.putString("Arm Status", "GOING UP");
  }

  /**
   * Stop the arm and hold position.
   */
  public void stop() {
    m_motor.setControl(m_dutyCycle.withOutput(0));
    m_isMoving = false;
  }

  /**
   * Emergency stop - resets the state to unknown.
   * Use this if something goes wrong.
   */
  public void forceStop() {
    m_motor.setControl(m_dutyCycle.withOutput(0));
    m_isMoving = false;
    m_state = DeployState.UNKNOWN;
    SmartDashboard.putString("Arm Status", "FORCE STOPPED");
  }

  // ========== INFO GETTERS ==========

  /** Get the current arm state */
  public DeployState getState() {
    return m_state;
  }

  /** Is the arm all the way down? */
  public boolean isDown() {
    return m_state == DeployState.DOWN;
  }

  /** Is the arm all the way up? */
  public boolean isUp() {
    return m_state == DeployState.UP;
  }

  /** Is the arm currently moving? */
  public boolean isMoving() {
    return m_isMoving;
  }

  /** Current amps being drawn */
  public double getCurrentAmps() {
    return m_motor.getSupplyCurrent().getValueAsDouble();
  }

  /** Current RPM (absolute value) */
  public double getRPM() {
    return Math.abs(m_motor.getVelocity().getValueAsDouble() * 60.0);
  }

  // ========== PERIODIC - RUNS EVERY 20MS ==========

  @Override
  public void periodic() {
    double amps = getCurrentAmps();
    double rpm = getRPM();

    // Check for stall while moving
    if (m_isMoving) {
      // A stall is when current is high but velocity is low
      boolean stalling = amps > MotorConstants.kDeployStallCurrentThreshold &&
                         rpm < MotorConstants.kDeployStallVelocityThreshold;

      if (stalling) {
        if (!m_highCurrentDetected) {
          // Just started stalling - start the timer
          m_highCurrentDetected = true;
          m_stallStartTime = Timer.getFPGATimestamp();
        } else if (Timer.getFPGATimestamp() - m_stallStartTime >= MotorConstants.kDeployStallTimeThreshold) {
          // Been stalling long enough - we hit the stop!
          m_stallDetected = true;
          stop();

          if (m_state == DeployState.MOVING_DOWN) {
            m_state = DeployState.DOWN;
            System.out.println("[ARM] Hit floor - locked DOWN");
            SmartDashboard.putString("Arm Status", "DOWN");
          } else if (m_state == DeployState.MOVING_UP) {
            m_state = DeployState.UP;
            m_motor.setPosition(0);  // Reset position at top
            System.out.println("[ARM] Hit top - locked UP");
            SmartDashboard.putString("Arm Status", "UP");
          }
        }
      } else {
        // Not stalling - reset the detection
        m_highCurrentDetected = false;
      }
    }

    // Update SmartDashboard
    SmartDashboard.putString("Arm State", m_state.toString());
    SmartDashboard.putNumber("Arm Amps", amps);
    SmartDashboard.putNumber("Arm RPM", rpm);
    SmartDashboard.putBoolean("Arm Moving", m_isMoving);
    SmartDashboard.putBoolean("Arm Stalled", m_stallDetected);
    SmartDashboard.putBoolean("Arm Alive", m_motor.isAlive());
  }
}
