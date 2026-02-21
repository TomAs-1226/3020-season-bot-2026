// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * ============================================================================
 *                       TEAM 3020 - MAIN ROBOT
 * ============================================================================
 *
 * This is the main file that runs on the roboRIO. When you turn on the robot,
 * this is what starts everything up.
 *
 * You probably won't need to change this file much. Most of the good stuff
 * is in RobotContainer.java - that's where the buttons and subsystems are.
 *
 * ROBOT MODES:
 *   DISABLED = Robot is on but waiting (before match starts)
 *   AUTO     = First 15 seconds, robot drives itself
 *   TELEOP   = Driver control - this is most of the match
 *   TEST     = For testing things in the pits
 *
 * @author Baichen Yu
 * ============================================================================
 */
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;

  /**
   * This runs once when the robot turns on.
   */
  public Robot() {
    System.out.println("============================================");
    System.out.println("        TEAM 3020 CODE STARTING");
    System.out.println("============================================");

    m_robotContainer = new RobotContainer();

    System.out.println("[TEAM 3020] Robot ready!");
  }

  /**
   * This runs every 20 milliseconds no matter what.
   * It's what makes all the commands actually work.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  // ===== DISABLED =====
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  // ===== AUTONOMOUS =====
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      System.out.println("[TEAM 3020] Auto starting!");
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  // ===== TELEOP =====
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    System.out.println("[TEAM 3020] TELEOP - GO GO GO!");
  }

  @Override
  public void teleopPeriodic() {}

  // ===== TEST =====
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  // ===== SIMULATION =====
  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
