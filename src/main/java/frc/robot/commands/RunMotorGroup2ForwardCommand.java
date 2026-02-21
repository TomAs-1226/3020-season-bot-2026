// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MotorGroup2Subsystem;

/**
 * =============================================================================
 * TEAM 3020 - SHOOTER FORWARD COMMAND
 * =============================================================================
 *
 * This command runs the shooter motors (Motor Group 2) at full speed forward
 * to launch game pieces toward the target.
 *
 * CONTROLS:
 *   - Button: L1 (Left bumper) on PS5 controller
 *   - Action: Hold to spin up shooter, release to stop
 *
 * MOTORS CONTROLLED:
 *   - CAN ID 3: Kraken X60 (left top)
 *   - CAN ID 4: Kraken X60 (left bottom)
 *   - CAN ID 5: Kraken X60 (right top)
 *   - CAN ID 6: Kraken X60 (right bottom)
 *
 * HOW IT WORKS:
 *   - All 4 shooter motors spin at matched velocity for consistent shots
 *   - Uses closed-loop velocity control (PID) for precise RPM
 *   - Target RPM adjustable from SmartDashboard
 *   - Motors ramp up smoothly to prevent brownouts
 *
 * TIP: Let the shooter spin up fully before feeding game pieces!
 *
 * COPIED FROM: Team 5805 Alphabot (sister team)
 * =============================================================================
 */
public class RunMotorGroup2ForwardCommand extends Command {

  private final MotorGroup2Subsystem m_subsystem;

  /**
   * Creates the shooter forward command.
   *
   * @param subsystem The shooter subsystem (Motor Group 2)
   */
  public RunMotorGroup2ForwardCommand(MotorGroup2Subsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    // Nothing special needed on start
  }

  @Override
  public void execute() {
    // Run shooter motors forward at full speed
    m_subsystem.runForward();
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
