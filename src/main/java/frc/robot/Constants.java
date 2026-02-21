// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * ============================================================================
 *                    TEAM 3020 - ROBOT CONSTANTS
 * ============================================================================
 *
 * Hey! This file has all the numbers that make our robot work.
 *
 * If something isn't working right, this is probably where you need to look.
 * All the CAN IDs, power levels, and settings are here.
 *
 * QUICK REFERENCE:
 *   - Motor CAN IDs: 1-8
 *   - Controller: PS5 on port 0
 *   - Intake power: 75%
 *   - Shooter RPM: adjustable via SmartDashboard
 *
 * @author Baichen Yu
 * ============================================================================
 */
public final class Constants {

  /**
   * Controller settings.
   * We use a PS5 controller plugged into port 0.
   */
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  /**
   * All the motor settings live here.
   *
   * OUR MOTORS:
   *   CAN 1 & 2 = Feeder (pushes game pieces to shooter)
   *   CAN 3,4,6,7 = Shooter (4 motors that spin really fast)
   *   CAN 5 = Intake roller (grabs game pieces)
   *   CAN 8 = Intake arm (swings up/down)
   */
  public static class MotorConstants {

    // ----- FEEDER MOTORS (CAN 1 & 2) -----
    // These push game pieces into the shooter
    public static final int kMotorGroup1Motor1ID = 1;
    public static final int kMotorGroup1Motor2ID = 2;
    public static final double kMotorGroup1PowerPercent = 50.0;  // How fast (0-100%)

    // ----- SHOOTER MOTORS (CAN 3, 4, 6, 7) -----
    // Four motors that spin up to launch game pieces
    public static final int kMotorGroup2Motor1ID = 3;
    public static final int kMotorGroup2Motor2ID = 4;
    public static final int kMotorGroup2Motor3ID = 6;
    public static final int kMotorGroup2Motor4ID = 7;
    public static final String kMotorGroup2CANBus = "";  // Empty = RIO bus
    public static final double kMotorGroup2TargetRPM = 2000.0;  // Target speed
    public static final double kMotorGroup2TargetRPS = kMotorGroup2TargetRPM / 60.0;

    // Shooter idle - keeps it spinning slowly for faster spinup
    public static final double kShooterIdleRPM = 500.0;
    public static final boolean kUseShooterIdle = true;

    // ----- INTAKE ROLLER (CAN 5) -----
    // Spins to grab game pieces from the floor
    public static final int kMotorGroup3MotorID = 5;
    public static final String kMotorGroup3CANBus = "";
    public static final double kIntakePowerPercent = 75.0;       // Intake speed
    public static final double kIntakeReversePowerPercent = 50.0; // Eject speed
    public static final boolean kIntakeUseBrakeMode = true;

    // ----- INTAKE ARM (CAN 8) -----
    // Swings the intake down to the floor and back up
    public static final int kMotorGroup4MotorID = 8;
    public static final String kMotorGroup4CANBus = "";
    public static final double kDeployDownPowerPercent = 50.0;  // Speed going down
    public static final double kDeployUpPowerPercent = 60.0;    // Speed going up
    public static final boolean kDeployUseBrakeMode = true;

    // Stall detection - how we know the arm hit the stop
    public static final double kDeployStallCurrentThreshold = 35.0;  // Amps
    public static final double kDeployStallVelocityThreshold = 50.0; // RPM
    public static final double kDeployStallTimeThreshold = 0.1;      // Seconds

    // ----- CURRENT LIMITS -----
    // These protect the motors from burning out
    public static final double kKrakenCurrentLimit = 80.0;    // Kraken X60
    public static final double kKraken44CurrentLimit = 40.0;  // Kraken X44
    public static final double kMotorGroup1CurrentLimitOverride = 120.0;  // Feeder needs more

    // ----- PID TUNING -----
    // Don't touch these unless you know what you're doing!
    public static final double kKrakenVelocityKp = 0.01;
    public static final double kKrakenVelocityKi = 0.0;
    public static final double kKrakenVelocityKd = 0.0;
    public static final double kKrakenVelocityKv = 0.12;
    public static final double kKrakenVelocityKs = 0.15;
    public static final double kKrakenVelocityKa = 0.01;

    // ----- GENERAL SETTINGS -----
    public static final double kRampRate = 0.25;  // Seconds to reach full speed
    public static final boolean kUseBrakeMode = false;  // Coast when stopped

    // Helper methods for current limits
    public static double getMotorGroup1CurrentLimit() {
      return kMotorGroup1CurrentLimitOverride;
    }

    public static double getMotorGroup2CurrentLimit() {
      return kKrakenCurrentLimit;
    }

    // Helper methods for PID (all motors use same values)
    public static double getMotorGroup1Kp() { return kKrakenVelocityKp; }
    public static double getMotorGroup2Kp() { return kKrakenVelocityKp; }
    public static double getMotorGroup1Kv() { return kKrakenVelocityKv; }
    public static double getMotorGroup2Kv() { return kKrakenVelocityKv; }
    public static double getMotorGroup1Ks() { return kKrakenVelocityKs; }
    public static double getMotorGroup2Ks() { return kKrakenVelocityKs; }
    public static double getMotorGroup1Ka() { return kKrakenVelocityKa; }
    public static double getMotorGroup2Ka() { return kKrakenVelocityKa; }
  }

  /**
   * Battery and power settings.
   * The robot watches battery voltage and slows down if it's getting low.
   */
  public static class PowerConstants {
    public static final double kNominalVoltage = 12.5;    // Full battery
    public static final double kWarningVoltage = 11.5;    // Getting low
    public static final double kCriticalVoltage = 11.0;   // Very low
    public static final double kEmergencyVoltage = 10.5;  // About to brown out!

    // How much to reduce power when battery is low
    public static final double kCriticalModeIntakeScale = 0.8;   // 80% power
    public static final double kEmergencyModeIntakeScale = 0.6;  // 60% power
  }
}
