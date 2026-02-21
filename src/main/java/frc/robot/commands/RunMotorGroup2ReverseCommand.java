// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MotorGroup2Subsystem;

/**
 * =============================================================================
 * TEAM 3020 - SHOOTER REVERSE COMMAND
 * =============================================================================
 *
 * This command runs the shooter motors (Motor Group 2) in reverse.
 * Useful for clearing jams or pulling game pieces back out.
 *
 * CONTROLS:
 *   - Button: R1 (Right bumper) on PS5 controller
 *   - Action: Hold to reverse shooter, release to stop
 *
 * MOTORS CONTROLLED:
 *   - CAN ID 3: Kraken X60 (left top)
 *   - CAN ID 4: Kraken X60 (left bottom)
 *   - CAN ID 5: Kraken X60 (right top)
 *   - CAN ID 6: Kraken X60 (right bottom)
 *
 * WHEN TO USE:
 *   - Game piece is stuck in the shooter
 *   - Need to back a game piece out after a failed shot
 *   - Testing shooter reverse direction
 *
 * WARNING: Don't run reverse for too long while game piece is loaded!
 *
 * COPIED FROM: Team 5805 Alphabot (sister team)
 * =============================================================================
 */
public class RunMotorGroup2ReverseCommand extends Command {

  private final MotorGroup2Subsystem m_subsystem;

  /**
   * Creates the shooter reverse command.
   *
   * @param subsystem The shooter subsystem (Motor Group 2)
   */
  public RunMotorGroup2ReverseCommand(MotorGroup2Subsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    // Nothing special needed on start
  }

  @Override
  public void execute() {
    // Run shooter motors in reverse
    m_subsystem.runReverse();
  }

  @Override
  public void end(boolean interrupted) {
    // Stop shooter when button is released
    m_subsystem.stopMotors();
  }

  @Override
  public boolean isFinished() {
    // Runs continuously while button is held
    return false;
  }
}
