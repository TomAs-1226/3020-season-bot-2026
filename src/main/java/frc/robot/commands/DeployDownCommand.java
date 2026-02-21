// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeDeploySubsystem;

/**
 * =============================================================================
 * TEAM 3020 - INTAKE ARM DOWN COMMAND
 * =============================================================================
 *
 * This command deploys the intake arm down to the floor to grab game pieces.
 * Uses stall detection - automatically stops when the arm hits the floor.
 *
 * CONTROLS:
 *   - Button: D-pad DOWN on PS5 controller
 *   - Action: Press once to deploy arm (auto-stops at floor)
 *
 * MOTOR CONTROLLED:
 *   - CAN ID 8: Kraken X60 (intake deploy arm)
 *
 * HOW IT WORKS:
 *   1. Motor starts spinning to push arm down
 *   2. Stall detection monitors current draw and RPM
 *   3. When current is high but RPM is low = arm hit the floor
 *   4. Motor stops automatically and arm locks in place (brake mode)
 *
 * TIP: For the best experience, use AutoDeployIntakeCommand on Square button!
 *      That command deploys the arm AND starts the intake rollers automatically.
 *
 * SAFETY:
 *   - If interrupted while moving, the arm stops immediately
 *   - Arm stays in DOWN position for quick re-intake
 *
 * COPIED FROM: Team 5805 Alphabot (sister team)
 * =============================================================================
 */
public class DeployDownCommand extends Command {

  private final IntakeDeploySubsystem m_deploy;

  /**
   * Creates the arm deploy command.
   *
   * @param deploy The intake deploy subsystem
   */
  public DeployDownCommand(IntakeDeploySubsystem deploy) {
    m_deploy = deploy;
    addRequirements(deploy);
  }

  @Override
  public void initialize() {
    // Start moving arm down
    m_deploy.deployDown();
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
    // Command ends when arm reaches the DOWN position (stall detected at floor)
    return m_deploy.isDown();
  }
}
