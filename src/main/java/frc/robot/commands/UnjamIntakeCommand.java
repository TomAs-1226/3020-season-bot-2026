// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeDeploySubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * ============================================================================
 *                    TEAM 3020 - UNJAM INTAKE COMMAND
 * ============================================================================
 *
 * Clears an intake jam by:
 *   1. Nudging the arm UP a small amount (using the motor's internal encoder)
 *   2. Then nudging it back DOWN to original position
 *   Throughout: intake roller spins in reverse to back the jam out
 *
 * The movement uses the TalonFX internal encoder (rotations), not an external
 * sensor.  Typical nudge = 0.5 motor rotations ≈ small angle change at output.
 *
 * BUTTON: R3 (press right stick)
 * ============================================================================
 */
public class UnjamIntakeCommand extends Command {

    private static final double ARM_NUDGE_ROTATIONS = 0.5;   // motor shaft rotations to move
    private static final double ARM_POWER_UP        = -0.25; // fraction — up = negative
    private static final double ARM_POWER_DOWN      =  0.25; // fraction — down = positive
    private static final double INTAKE_REVERSE_POWER = -0.50; // reverse at 50%

    private final IntakeDeploySubsystem m_arm;
    private final IntakeSubsystem       m_intake;

    private double m_startPosition;
    private boolean m_movingUp; // true = phase 1 (going up), false = phase 2 (returning down)

    public UnjamIntakeCommand(IntakeDeploySubsystem arm, IntakeSubsystem intake) {
        m_arm    = arm;
        m_intake = intake;
        addRequirements(arm, intake);
    }

    @Override
    public void initialize() {
        m_startPosition = m_arm.getPositionRotations();
        m_movingUp      = true;
        // Immediately start reversing intake roller
        m_intake.runAtPower(INTAKE_REVERSE_POWER);
    }

    @Override
    public void execute() {
        double currentPos = m_arm.getPositionRotations();

        if (m_movingUp) {
            // Phase 1: nudge arm up until we've moved the target amount
            m_arm.runRaw(ARM_POWER_UP);
            if (m_startPosition - currentPos >= ARM_NUDGE_ROTATIONS) {
                // Reached target upward position — now go back down
                m_movingUp = false;
            }
        } else {
            // Phase 2: return arm to original position
            m_arm.runRaw(ARM_POWER_DOWN);
        }
    }

    @Override
    public boolean isFinished() {
        if (m_movingUp) return false;
        // Done when arm is back to (or past) start position
        return m_arm.getPositionRotations() >= m_startPosition;
    }

    @Override
    public void end(boolean interrupted) {
        m_arm.stop();
        m_intake.stop();
    }
}
