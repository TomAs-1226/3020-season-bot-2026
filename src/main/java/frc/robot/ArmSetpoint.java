// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.MotorConstants;

/**
 * Named arm setpoints for the intake deploy mechanism.
 *
 * <p>Values are in motor shaft rotations (1:36 gear ratio).
 * Position 0 = arm fully UP (stowed at startup). Going DOWN is positive.
 *
 * <p>The DEPLOYED position is also tunable at runtime via the Shuffleboard
 * "Arm" tab — the live Shuffleboard value overrides the default here.
 *
 * <p>To add new setpoints (e.g. a partial stow position):
 *   1. Add a constant in {@link MotorConstants} (e.g. kArmPartialStowRotations).
 *   2. Add a new enum entry below pointing to that constant.
 *   3. Call {@code moveToSetpoint(ArmSetpoint.PARTIAL_STOW)} from a command.
 */
public enum ArmSetpoint {

    /** Arm fully retracted / stowed inside the robot frame. */
    STOWED(MotorConstants.kArmStowedRotations),

    /** Arm fully deployed down to the floor for game piece intake. */
    DEPLOYED(MotorConstants.kArmDeployedRotations);

    // Motor shaft rotations for this setpoint (default / compile-time value).
    // The DEPLOYED position may be overridden at runtime via Shuffleboard.
    private final double motorRotations;

    ArmSetpoint(double motorRotations) {
        this.motorRotations = motorRotations;
    }

    /**
     * Returns the default motor shaft rotation target for this setpoint.
     * For DEPLOYED, prefer reading the live Shuffleboard value in the subsystem.
     */
    public double getMotorRotations() {
        return motorRotations;
    }
}
