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
 * CAN ID MAP (all on RIO bus ""):
 *   Flywheel 1   : 30  (Kraken X60 FOC) - leader
 *   Flywheel 2   : 31  (Kraken X60 FOC) - follower
 *   Feeder Hopper: 40  (Kraken X60 FOC)
 *   Feeder Roller: 41  (Kraken X60 FOC)
 *   Intake Roller: 45  (Falcon 500 - NO FOC)
 *   Intake Arm   : 46  (Kraken X60 FOC)
 *   Swerve FL    : steer=11, drive=12, encoder=13
 *   Swerve BL    : steer=14, drive=15, encoder=16
 *   Swerve BR    : steer=17, drive=18, encoder=19
 *   Swerve FR    : steer=20, drive=21, encoder=22
 *   Pigeon2      : 23
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

  public static class MotorConstants {

    // ----- FEEDER MOTORS -----
    // Hopper (CAN 40) + Feed Roller (CAN 41) - both Kraken X60 FOC
    public static final int kMotorGroup1Motor1ID = 40; // Hopper
    public static final int kMotorGroup1Motor2ID = 41; // Feed roller
    public static final double kMotorGroup1HopperPowerPercent = 10.0; // Hopper — slow; tune up in Shuffleboard
    public static final double kMotorGroup1PowerPercent = 20.0;       // Feed Roller — tune up in Shuffleboard

    // ----- SHOOTER / FLYWHEEL MOTORS -----
    // CAN 30 (leader) and CAN 31 (follower) - Kraken X60 FOC
    public static final int kMotorGroup2Motor1ID = 30; // Flywheel leader
    public static final int kMotorGroup2Motor2ID = 31; // Flywheel follower
    public static final String kMotorGroup2CANBus = "";
    public static final double kMotorGroup2TargetRPM = 3000.0; // Tune via SmartDashboard
    public static final double kMotorGroup2TargetRPS = kMotorGroup2TargetRPM / 60.0;
    public static final double kShooterOnTargetToleranceRPM = 150.0; // within this = ready to feed

    // Shooter idle - keeps flywheel spinning for faster spinup
    public static final double kShooterIdleRPM = 500.0;
    public static final boolean kUseShooterIdle = true;

    // ----- INTAKE ROLLER (CAN 45) -----
    // FALCON 500 - does NOT support FOC, use withEnableFOC(false)
    public static final int kMotorGroup3MotorID = 45;
    public static final String kMotorGroup3CANBus = "";
    public static final double kIntakePowerPercent = 90.0;
    public static final double kIntakeReversePowerPercent = 50.0;
    public static final boolean kIntakeUseBrakeMode = true;

    // ----- INTAKE ARM (CAN 46) -----
    // Kraken X60 FOC, 1:36 gear ratio — arm position 0 = fully UP (stowed at startup)
    public static final int kMotorGroup4MotorID = 46;
    public static final String kMotorGroup4CANBus = "";
    public static final double kDeployDownPowerPercent = 50.0;
    public static final double kDeployUpPowerPercent = 60.0;
    public static final boolean kDeployUseBrakeMode = true;

    // Gear ratio — motor rotations per one full arm rotation (1:36)
    public static final double kArmGearRatio = 36.0;

    // Stall detection — stops arm when it hits the physical bumper hard stop.
    // Uses BOTH velocity AND current so deceleration approaching target doesn't false-trigger:
    //   At bumper: RPM near 0 AND amps high (motor fighting hard stop)
    //   Approaching target: RPM near 0 BUT amps low (MotionMagic braking gently)
    public static final double kDeployStallVelocityThreshold = 120.0; // RPM — below this = possibly stalled
    public static final double kDeployStallCurrentThreshold  =  8.0;  // Amps — above this = motor fighting something
    public static final double kDeployStallTimeThreshold = 0.15;      // seconds of combined stall before stop
    public static final double kDeployStallStartupDelay = 0.3;        // seconds after move starts before stall detection active
    public static final double kDeployMoveTimeoutSeconds = 3.0;      // seconds — hard timeout for any arm move (safety net if stall detection fails)
    public static final double kArmStallBackoffRotations = 0.3;      // motor rotations to back off from bumper after stall — prevents motor pushing into hard stop

    // Arm software limits (motor shaft rotations)
    // Position 0 = arm fully UP (home). Going DOWN = positive.
    // kArmForwardSoftLimit must be >= kArmDeployedRotations to not interfere.
    public static final double kArmForwardSoftLimit = 28.0;  // absolute max — stays above down target
    public static final double kArmReverseSoftLimit = -1.0;  // safety buffer above home
    public static final boolean kArmSoftLimitsEnable = true;

    // ----- ARM NAMED SETPOINTS (motor shaft rotations, 0 = arm fully UP) -----
    // HOW TO TUNE kArmDeployedRotations:
    //   1. Run arm down slowly with D-pad, read "Arm Position" in Shuffleboard "Arm" tab.
    //   2. Enter the target rotation count into "Arm Down Position (rot)" in Shuffleboard.
    //   3. Value auto-saves to Preferences — no redeploy needed.
    public static final double kArmStowedRotations   = 0.0;   // arm fully UP (home)
    public static final double kArmDeployedRotations = 20.0;  // arm fully DOWN for floor intake — TUNE ME
    public static final double kArmPositionToleranceRotations = 0.5;  // within this = at target

    // ----- ARM GRAVITY FEEDFORWARD -----
    // Full control equation:  volts = kS + kV*velocity + kG*cos(theta)
    //   kS     = static friction — minimum voltage to overcome stiction
    //   kV     = velocity feedforward — proportional to target velocity (motor shaft RPS)
    //   kG     = gravity gain — peak voltage when the arm is horizontal (max gravity torque)
    //   theta  = arm angle measured from horizontal; cos(theta) automatically scales
    //            gravity compensation with actual arm angle (0° = horizontal = max comp,
    //            90° = fully vertical = zero comp)
    //
    // HOW TO TUNE kG:
    //   1. Finish tuning kS, kV, kP first so the arm moves reliably.
    //   2. Hold the arm horizontally, then set kG in Shuffleboard "Arm" tab.
    //   3. Increase kG until the arm holds horizontal without drooping.
    //   4. Confirm the arm holds at full-stow and deployed — cos(theta) handles scaling.
    //   Typical starting range: 0.3 – 0.8 V for a lightweight intake arm.
    //
    // HOW TO SET kArmZeroAngleDeg:
    //   This is the arm's physical angle above horizontal (in degrees) when the encoder
    //   reads 0 (arm fully stowed / UP).  Measure from your robot geometry:
    //     - Arm points straight up  → ~90°
    //     - Arm leans slightly past vertical → ~95–100°
    //   This only affects gravity compensation accuracy; all setpoints still use motor rot.
    public static final double kArmKg = 0.0;           // V — gravity gain (TUNE ME, start ~0.3–0.8)
    public static final double kArmZeroAngleDeg = 90.0; // degrees above horizontal at encoder zero (TUNE ME)

    // ----- ARM MOTION MAGIC SLOT 0 PID/FF -----
    // Slot 0 PID gains (MotionMagicVoltage — units VOLTS)
    // Note: kG is NOT in Slot 0; it is applied manually each cycle via withFeedForward()
    //       so that the cos(theta) angle compensation is live and accurate.
    public static final double kArmKp = 2.0;    // V/rotation  — position error correction
    public static final double kArmKi = 0.0;
    public static final double kArmKd = 0.0;
    public static final double kArmKv = 0.12;   // V·s/rotation — velocity feedforward
    public static final double kArmKs = 0.25;   // V            — static friction

    // MotionMagic trapezoidal profile (motor shaft)
    public static final double kArmCruiseVelocity          = 15.0; // RPS — normal stow/deploy speed
    public static final double kArmShootStowCruiseVelocity =  4.0; // RPS — slow push while shooting (~40 deg/s arm)
    public static final double kArmAcceleration            = 25.0; // RPS/s — reaches cruise in 0.6 s

    // ----- CURRENT LIMITS -----
    public static final double kKrakenCurrentLimit = 80.0;
    public static final double kKraken44CurrentLimit = 40.0;
    public static final double kFalconCurrentLimit = 40.0; // Falcon 500 supply limit
    public static final double kMotorGroup1CurrentLimitOverride = 40.0; // Feeder — was 80A (caused voltage sag killing shooter spinup)

    // ----- SHOOTER PID / FF TUNING  (VelocityVoltage + FOC) -----
    // Units are VOLTS, not amps.  Tune live via SmartDashboard; commit final values here.
    //
    // How each gain controls "catch-up":
    //   kP  — main catch-up term: volts applied per RPS of error.  Raise to spin up faster.
    //   kV  — feedforward: volts per RPS of target.  Pre-loads voltage so kP only trims error.
    //   kS  — static friction: minimum volts to get the flywheel moving at all.
    //   kA  — acceleration feedforward: extra volts per (RPS/s) during spinup.  Usually 0 to start.
    //
    // Safe starting points for Kraken X60 FOC flywheel; raise kP if it spins up too slowly.
    public static final double kShooterKp = 0.4;    // V / RPS  — raised from 0.1; needs correction authority to hold RPM under load
    public static final double kShooterKi = 0.0;
    public static final double kShooterKd = 0.0;
    public static final double kShooterKv = 0.12;   // V·s / rot — steady-state feedforward
    public static final double kShooterKs = 0.1;    // V         — static friction
    public static final double kShooterKa = 0.0;    // V·s²/rot  — acceleration FF (start at 0)

    // Legacy voltage-mode gains kept so ArmCharacterizeCommand still compiles
    public static final double kKrakenVelocityKp = 0.3;
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
    public static final double kWarningVoltage = 11.0;    // Getting low (was 11.5)
    public static final double kCriticalVoltage = 10.5;   // Very low (was 11.0)
    public static final double kEmergencyVoltage = 10.0;  // About to brown out (was 10.5)!

    // How much to reduce power when battery is low
    public static final double kCriticalModeIntakeScale = 0.8;   // 80% power
    public static final double kEmergencyModeIntakeScale = 0.6;  // 60% power
  }
}
