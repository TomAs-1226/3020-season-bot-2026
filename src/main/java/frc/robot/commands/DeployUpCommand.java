// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeDeploySubsystem;

/**
 * =============================================================================
 * TEAM 3020 - INTAKE ARM UP COMMAND
 * =============================================================================
 *
 * This command retracts the intake arm back up into the robot frame.
 * Uses stall detection - automatically stops when the arm hits the top.
 *
 * CONTROLS:
 *   - Button: D-pad UP on PS5 controller
 *   - Action: Press once to retract arm (auto-stops at top)
 *
 * MOTOR CONTROLLED:
 *   - CAN ID 8: Kraken X60 (intake deploy arm)
 *
 * HOW IT WORKS:
 *   1. Motor starts spinning to pull arm up
 *   2. Stall detection monitors current draw and RPM
 *   3. When current is high but RPM is low = arm hit the top stop
 *   4. Motor stops automatically and arm locks in place (brake mode)
 *
 * SAFETY:
 *   - If interrupted while moving, the arm stops immediately
 *   - Position is reset to 0 when arm reaches top (home position)
 *
 * COPIED FROM: Team 5805 Alphabot (sister team)
 * =============================================================================
 */
public class DeployUpCommand extends Command {

  private final IntakeDeploySubsystem m_deploy;

  /**
   * Creates the arm retract command.
   *
   * @param deploy The intake deploy subsystem
   */
  public DeployUpCommand(IntakeDeploySubsystem deploy) {
    m_deploy = deploy;
    addRequirements(deploy);
  }

  @Override
  public void initialize() {
    // Start moving arm up
    m_deploy.deployUp();
  }

  @Override
  public void end(boolean interrupted) {
    // If interrupted while still moving, stop the motor
    if (interrupted && m_deploy.isMoving()) {
      m_deploy.stop();
    }
  }

  @Override
  public boolean isFinished() {
    // Command ends when arm reaches the UP position (stall detected at top)
    return m_deploy.isUp();
  }
}
