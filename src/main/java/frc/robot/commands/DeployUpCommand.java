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
  public void execute() {
    // Re-apply position setpoint every cycle so kG*cos(theta) gravity FF stays accurate
    // as the arm fights gravity through its arc. Does NOT reset stall detection timing.
    m_deploy.refreshFeedForward();
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
    // isAtTarget() = encoder within tolerance (normal arrival at stow position)
    // isUp()       = state set to UP (covers stall-at-top-stop where encoder may
    //                sit slightly outside tolerance due to physical hard stop)
    return m_deploy.isAtTarget() || m_deploy.isUp();
  }
}
