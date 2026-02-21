// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * =============================================================================
 * TEAM 3020 - INTAKE EJECT COMMAND
 * =============================================================================
 *
 * This command reverses the intake rollers to spit out game pieces.
 *
 * CONTROLS:
 *   - Button: Circle on PS5 controller
 *   - Action: Hold to eject, release to stop
 *
 * MOTOR CONTROLLED:
 *   - CAN ID 7: Kraken X44 (intake roller, reversed)
 *
 * WHEN TO USE:
 *   - Grabbed the wrong game piece (wrong color, wrong type)
 *   - Game piece is jammed in the intake
 *   - Need to pass a game piece to a teammate
 *   - Positioning game piece for a better grip
 *
 * TIP: The intake arm stays wherever it is - this just reverses the rollers.
 *      Lower the arm first if you need to drop the game piece on the floor.
 *
 * COPIED FROM: Team 5805 Alphabot (sister team)
 * =============================================================================
 */
public class RunIntakeReverseCommand extends Command {

  private final IntakeSubsystem m_intake;

  /**
   * Creates the intake eject command.
   *
   * @param intake The intake subsystem
   */
  public RunIntakeReverseCommand(IntakeSubsystem intake) {
    m_intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    System.out.println("[TEAM 3020 INTAKE] Eject started");
  }

  @Override
  public void execute() {
    // Run intake rollers in reverse to eject game piece
    m_intake.runReverse();
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
    System.out.println("[TEAM 3020 INTAKE] Eject stopped" + (interrupted ? " (interrupted)" : ""));
  }

  @Override
  public boolean isFinished() {
    // Runs while button is held
    return false;
  }
}
