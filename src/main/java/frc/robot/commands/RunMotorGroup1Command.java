// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MotorGroup1Subsystem;

/**
 * =============================================================================
 * TEAM 3020 - FEEDER COMMAND
 * =============================================================================
 *
 * This command runs the feeder motors (Motor Group 1) to push game pieces
 * from the intake into the shooter.
 *
 * CONTROLS:
 *   - Button: Cross (X) on PS5 controller
 *   - Action: Hold to run feeder, release to stop
 *
 * MOTORS CONTROLLED:
 *   - CAN ID 1: Kraken X60 (clockwise)
 *   - CAN ID 2: Kraken X60 (counter-clockwise)
 *
 * HOW IT WORKS:
 *   - While the driver holds the Cross button, both motors spin to push
 *     game pieces toward the shooter
 *   - When released, motors stop and brake (brake mode enabled)
 *   - Power level can be adjusted live from SmartDashboard "Group1 Power %"
 *
 * COPIED FROM: Team 5805 Alphabot (sister team)
 * =============================================================================
 */
public class RunMotorGroup1Command extends Command {

  private final MotorGroup1Subsystem m_subsystem;

  /**
   * Creates the feeder command.
   *
   * @param subsystem The feeder subsystem (Motor Group 1)
   */
  public RunMotorGroup1Command(MotorGroup1Subsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    // Nothing special needed on start
  }

  @Override
  public void execute() {
    // Run the feeder motors every loop while button is held
    m_subsystem.runMotors();
  }

  @Override
  public void end(boolean interrupted) {
    // Stop motors when button is released or command is interrupted
    m_subsystem.stopMotors();
  }

  @Override
  public boolean isFinished() {
    // Never finishes on its own - runs while button is held
    return false;
  }
}
