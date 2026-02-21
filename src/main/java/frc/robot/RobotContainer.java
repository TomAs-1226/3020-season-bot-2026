// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoDeployIntakeCommand;
import frc.robot.commands.DeployDownCommand;
import frc.robot.commands.DeployUpCommand;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.RunIntakeReverseCommand;
import frc.robot.commands.RunMotorGroup1Command;
import frc.robot.commands.RunMotorGroup2ForwardCommand;
import frc.robot.commands.RunMotorGroup2ReverseCommand;
import frc.robot.subsystems.IntakeDeploySubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MotorGroup1Subsystem;
import frc.robot.subsystems.MotorGroup2Subsystem;
import frc.robot.subsystems.PowerManagementSubsystem;
import frc.robot.subsystems.TelemetrySubsystem;

/**
 * ============================================================================
 *                    TEAM 3020 - ROBOT CONTAINER
 * ============================================================================
 *
 * This is where everything comes together! All our subsystems are created here
 * and all the controller buttons are set up here.
 *
 * OUR SUBSYSTEMS:
 *   - Feeder (2 motors) - pushes game pieces to shooter
 *   - Shooter (4 motors) - launches game pieces
 *   - Intake (1 motor) - grabs game pieces from floor
 *   - Intake Arm (1 motor) - swings intake up/down
 *   - Power Management - watches battery voltage
 *   - Telemetry - shows info on dashboard
 *
 * CONTROLLER: PS5 DualSense on port 0
 *
 * BUTTON LAYOUT:
 *   Cross (X)  = Feeder (hold to run)
 *   L1         = Shooter forward
 *   R1         = Shooter reverse
 *   Square     = Auto intake (deploys arm + runs rollers)
 *   Circle     = Eject (reverse intake)
 *   D-pad UP   = Arm up
 *   D-pad DOWN = Arm down
 *   Triangle   = Manual intake (just rollers, no arm)
 *
 * @author Baichen Yu
 * ============================================================================
 */
public class RobotContainer {

  // ===== OUR SUBSYSTEMS =====

  /** Feeder - pushes game pieces into the shooter */
  private final MotorGroup1Subsystem m_feeder = new MotorGroup1Subsystem();

  /** Shooter - 4 motors that spin up to launch game pieces */
  private final MotorGroup2Subsystem m_shooter = new MotorGroup2Subsystem();

  /** Intake rollers - spin to grab game pieces */
  private final IntakeSubsystem m_intake = new IntakeSubsystem();

  /** Intake arm - swings down to the floor */
  private final IntakeDeploySubsystem m_intakeArm = new IntakeDeploySubsystem();

  /** Power management - watches battery and slows down if needed */
  private final PowerManagementSubsystem m_power = new PowerManagementSubsystem();

  /** Telemetry - shows battery and loop time on dashboard */
  private final TelemetrySubsystem m_telemetry = new TelemetrySubsystem();

  // ===== CONTROLLER =====
  private final CommandPS5Controller m_controller =
      new CommandPS5Controller(OperatorConstants.kDriverControllerPort);

  /**
   * Creates everything and sets up the buttons.
   */
  public RobotContainer() {
    System.out.println("============================================");
    System.out.println("        TEAM 3020 ROBOT STARTING UP");
    System.out.println("============================================");

    // Connect subsystems that need to talk to each other
    m_intake.setPowerManagement(m_power);

    // Set up all the button bindings
    configureBindings();

    System.out.println("[TEAM 3020] Ready to rock!");
    System.out.println("[TEAM 3020] Controller on port " + OperatorConstants.kDriverControllerPort);
  }

  /**
   * Sets up all the controller buttons.
   *
   * PS5 CONTROLLER:
   *
   *       [L1]                [R1]
   *       [L2]                [R2]
   *
   *  [D-PAD]     [OPTIONS]  [TRIANGLE]
   *    UP                    [SQUARE] [CIRCLE]
   *  L    R       [CREATE]   [CROSS]
   *   DOWN
   *
   *       [L3]                [R3]
   *      (stick)             (stick)
   */
  private void configureBindings() {

    // ===== FEEDER (Cross button) =====
    // Hold Cross to push game pieces toward the shooter
    m_controller.cross()
        .whileTrue(new RunMotorGroup1Command(m_feeder));

    // ===== SHOOTER (L1 / R1) =====
    // L1 = Spin up shooter (forward)
    // R1 = Reverse shooter (for clearing jams)
    m_controller.L1()
        .whileTrue(new RunMotorGroup2ForwardCommand(m_shooter));
    m_controller.R1()
        .whileTrue(new RunMotorGroup2ReverseCommand(m_shooter));

    // ===== INTAKE (Square / Circle / Triangle) =====
    // Square = Smart intake - auto-deploys arm then runs rollers
    // Circle = Eject - reverses rollers to spit out game piece
    // Triangle = Manual intake - just runs rollers (if arm already down)
    m_controller.square()
        .whileTrue(new AutoDeployIntakeCommand(m_intake, m_intakeArm));
    m_controller.circle()
        .whileTrue(new RunIntakeReverseCommand(m_intake));
    m_controller.triangle()
        .whileTrue(new RunIntakeCommand(m_intake));

    // ===== ARM CONTROL (D-pad) =====
    // D-pad Up = Retract arm back into robot
    // D-pad Down = Deploy arm to the floor
    m_controller.povUp()
        .whileTrue(new DeployUpCommand(m_intakeArm));
    m_controller.povDown()
        .whileTrue(new DeployDownCommand(m_intakeArm));

    System.out.println("[TEAM 3020] Buttons configured:");
    System.out.println("  Cross    = Feeder");
    System.out.println("  L1       = Shooter forward");
    System.out.println("  R1       = Shooter reverse");
    System.out.println("  Square   = Auto intake (arm + rollers)");
    System.out.println("  Circle   = Eject");
    System.out.println("  Triangle = Intake rollers only");
    System.out.println("  D-pad    = Arm up/down");
  }

  /**
   * Returns the command to run in autonomous.
   * Right now we don't have any auto - robot just sits there.
   */
  public Command getAutonomousCommand() {
    // TODO: Add autonomous routines later!
    return null;
  }
}
