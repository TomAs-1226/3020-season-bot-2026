// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * =============================================================================
 * TEAM 3020 - INTAKE COMMAND
 * =============================================================================
 *
 * This command spins the intake rollers to grab game pieces from the field.
 *
 * CONTROLS:
 *   - Button: Square on PS5 controller
 *   - Action: Hold to intake, release to stop
 *
 * MOTOR CONTROLLED:
 *   - CAN ID 7: Kraken X44 (intake roller)
 *
 * HOW IT WORKS:
 *   - Intake rollers spin inward to grab game pieces
 *   - Uses power scaling from PowerManagementSubsystem during low battery
 *   - Tracks total rotations and peak current for diagnostics
 *
 * TIP: For the best intake experience, use AutoDeployIntakeCommand instead!
 *      That command automatically lowers the arm AND runs the rollers.
 *
 * COPIED FROM: Team 5805 Alphabot (sister team)
 * =============================================================================
 */
public class RunIntakeCommand extends Command {

  private final IntakeSubsystem m_intake;

  /**
   * Creates the intake command.
   *
   * @param intake The intake subsystem
   */
  public RunIntakeCommand(IntakeSubsystem intake) {
    m_intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    System.out.println("[TEAM 3020 INTAKE] Intake started");
  }

  @Override
  public void execute() {
    // Run intake rollers to grab game pieces
    m_intake.runIntake();
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
    System.out.println("[TEAM 3020 INTAKE] Intake stopped" + (interrupted ? " (interrupted)" : ""));
  }

  @Override
  public boolean isFinished() {
    // Runs while button is held
    return false;
  }
}
