// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeDeploySubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * =============================================================================
 * TEAM 3020 - AUTO DEPLOY + INTAKE COMMAND (THE BEST ONE!)
 * =============================================================================
 *
 * This is the "smart" intake command that does everything automatically:
 *   1. Deploys the intake arm down (if it isn't already)
 *   2. Waits for arm to reach the floor
 *   3. Starts spinning intake rollers to grab game pieces
 *
 * CONTROLS:
 *   - Button: Square on PS5 controller
 *   - Action: HOLD to deploy arm and intake, RELEASE to stop intake
 *   - Note: Arm stays down after release for quick re-intake!
 *
 * MOTORS CONTROLLED:
 *   - CAN ID 7: Kraken X44 (intake roller)
 *   - CAN ID 8: Kraken X60 (intake arm deploy)
 *
 * WORKFLOW:
 *   1. Driver presses Square
 *   2. If arm is UP -> arm starts deploying, status shows "DEPLOYING..."
 *   3. When arm hits floor -> rollers start, status shows "INTAKING"
 *   4. If arm is already DOWN -> rollers start immediately
 *   5. Driver releases Square -> rollers stop, arm stays down
 *   6. Ready for next intake cycle without re-deploying!
 *
 * WHY USE THIS INSTEAD OF SEPARATE COMMANDS?
 *   - One button does everything
 *   - Can't accidentally run rollers before arm is down
 *   - Arm stays down for faster pickup cycles
 *   - SmartDashboard shows current status
 *
 * COPIED FROM: Team 5805 Alphabot (sister team)
 * =============================================================================
 */
public class AutoDeployIntakeCommand extends Command {

  private final IntakeSubsystem m_intake;
  private final IntakeDeploySubsystem m_deploy;
  private boolean m_deployStarted = false;

  /**
   * Creates the auto-deploy intake command.
   *
   * @param intake The intake rollers subsystem
   * @param deploy The intake arm deploy subsystem
   */
  public AutoDeployIntakeCommand(IntakeSubsystem intake, IntakeDeploySubsystem deploy) {
    m_intake = intake;
    m_deploy = deploy;
    addRequirements(intake, deploy);
  }

  @Override
  public void initialize() {
    m_deployStarted = false;

    // Check if arm needs to deploy first
    if (!m_deploy.isDown()) {
      // Arm isn't down yet - deploy it first
      m_deploy.deployDown();
      m_deployStarted = true;
      SmartDashboard.putString("Auto Intake", "DEPLOYING...");
      System.out.println("[TEAM 3020 AUTO-INTAKE] Deploying arm first");
    } else {
      // Arm already down - start intake immediately
      m_intake.runIntake();
      SmartDashboard.putString("Auto Intake", "INTAKING");
      System.out.println("[TEAM 3020 AUTO-INTAKE] Arm already down, intaking");
    }
  }

  @Override
  public void execute() {
    // Wait for deploy to finish, then start intake rollers
    if (m_deployStarted && m_deploy.isDown()) {
      m_intake.runIntake();
      m_deployStarted = false;  // Only trigger once
      SmartDashboard.putString("Auto Intake", "INTAKING");
      System.out.println("[TEAM 3020 AUTO-INTAKE] Arm deployed, now intaking");
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the rollers
    m_intake.stop();
    // NOTE: Arm stays down for quick re-intake!
    SmartDashboard.putString("Auto Intake", "STOPPED");
    System.out.println("[TEAM 3020 AUTO-INTAKE] Stopped" + (interrupted ? " (interrupted)" : ""));
  }

  @Override
  public boolean isFinished() {
    // Runs continuously until button is released
    return false;
  }
}
