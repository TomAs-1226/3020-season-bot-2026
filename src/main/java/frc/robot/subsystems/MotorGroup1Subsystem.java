// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

/**
 * ============================================================================
 *                    TEAM 3020 - FEEDER SUBSYSTEM
 * ============================================================================
 *
 * This is the feeder! It's the two motors that push game pieces from the
 * intake into the shooter.
 *
 * HOW IT WORKS:
 *   - Press Cross (X) to run the feeder
 *   - It pushes the game piece into the spinning shooter wheels
 *   - Make sure the shooter is spun up before feeding!
 *
 * MOTORS:
 *   - Motor 1 (CAN 1) = Kraken X60
 *   - Motor 2 (CAN 2) = Kraken X60
 *   - Both motors spin together
 *
 * TUNING:
 *   You can change the feeder speed on SmartDashboard:
 *   Look for "Group1 Power %" and adjust it (0-100%).
 *
 * @author Baichen Yu
 * ============================================================================
 */
public class MotorGroup1Subsystem extends SubsystemBase {

  // The two feeder motors
  private final TalonFX m_motor1;
  private final TalonFX m_motor2;

  // Duty cycle control - simple direct power control
  // EnableFOC gives better torque at low speeds
  private final DutyCycleOut m_motor1DutyCycle = new DutyCycleOut(0).withEnableFOC(true);
  private final DutyCycleOut m_motor2DutyCycle = new DutyCycleOut(0).withEnableFOC(true);

  // Current power level (0.0 to 1.0)
  private double m_powerPercent = MotorConstants.kMotorGroup1PowerPercent / 100.0;

  /**
   * Creates the feeder subsystem.
   * Sets up both motors with matching configuration.
   */
  public MotorGroup1Subsystem() {
    System.out.println("[FEEDER] Starting up...");

    // Connect to both motors on the CAN bus
    m_motor1 = new TalonFX(MotorConstants.kMotorGroup1Motor1ID, new CANBus(MotorConstants.kMotorGroup2CANBus));
    m_motor2 = new TalonFX(MotorConstants.kMotorGroup1Motor2ID, new CANBus(MotorConstants.kMotorGroup2CANBus));

    // Configure both motors
    configureTalonFX(m_motor1, InvertedValue.CounterClockwise_Positive);
    configureTalonFX(m_motor2, InvertedValue.CounterClockwise_Positive);

    // Put the power setting on SmartDashboard so drivers can adjust it
    SmartDashboard.putNumber("Group1 Power %", MotorConstants.kMotorGroup1PowerPercent);

    System.out.println("[FEEDER] 2 motors ready!");
    System.out.println("[FEEDER] Default power: " + MotorConstants.kMotorGroup1PowerPercent + "%");
    System.out.println("[FEEDER] Current limit: " + MotorConstants.getMotorGroup1CurrentLimit() + "A");
  }

  /**
   * Configures a single motor with current limits and output settings.
   */
  private void configureTalonFX(TalonFX motor, InvertedValue inverted) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Current limiting - feeder gets more current (120A)
    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.SupplyCurrentLimit = MotorConstants.getMotorGroup1CurrentLimit();
    currentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits = currentLimits;

    // Motor output settings
    MotorOutputConfigs motorOutput = new MotorOutputConfigs();
    motorOutput.Inverted = inverted;
    motorOutput.NeutralMode = MotorConstants.kUseBrakeMode ?
        NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.MotorOutput = motorOutput;

    // Ramp rate for smooth acceleration
    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = MotorConstants.kRampRate;

    // Try to apply config up to 3 times
    for (int i = 0; i < 3; i++) {
      var status = motor.getConfigurator().apply(config);
      if (status.isOK()) {
        break;
      }
    }
  }

  // ========== COMMANDS CALL THESE ==========

  /**
   * Run the feeder motors!
   * Uses the power setting from SmartDashboard.
   */
  public void runMotors() {
    // Read power from SmartDashboard and convert to 0.0-1.0
    m_powerPercent = SmartDashboard.getNumber("Group1 Power %", MotorConstants.kMotorGroup1PowerPercent) / 100.0;
    m_powerPercent = Math.max(0.0, Math.min(1.0, m_powerPercent));

    // Run both motors
    m_motor1.setControl(m_motor1DutyCycle.withOutput(m_powerPercent));
    m_motor2.setControl(m_motor2DutyCycle.withOutput(m_powerPercent));

    SmartDashboard.putBoolean("Group1 Running", true);
  }

  /**
   * Stop the feeder motors.
   */
  public void stopMotors() {
    m_motor1.setControl(m_motor1DutyCycle.withOutput(0));
    m_motor2.setControl(m_motor2DutyCycle.withOutput(0));
    SmartDashboard.putBoolean("Group1 Running", false);
  }

  // ========== PERIODIC - RUNS EVERY 20MS ==========

  @Override
  public void periodic() {
    // Motor speeds
    SmartDashboard.putNumber("Group1 Motor1 RPM", m_motor1.getVelocity().getValueAsDouble() * 60.0);
    SmartDashboard.putNumber("Group1 Motor2 RPM", m_motor2.getVelocity().getValueAsDouble() * 60.0);

    // Current draw
    SmartDashboard.putNumber("Group1 Motor1 Amps", m_motor1.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Group1 Motor2 Amps", m_motor2.getSupplyCurrent().getValueAsDouble());

    // Are motors working?
    SmartDashboard.putBoolean("Group1 Motor1 Alive", m_motor1.isAlive());
    SmartDashboard.putBoolean("Group1 Motor2 Alive", m_motor2.isAlive());
    SmartDashboard.putNumber("Group1 Power Applied", m_powerPercent * 100.0);
  }
}
