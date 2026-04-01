// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.IntakeDeploySubsystem;
import frc.robot.subsystems.MotorGroup2Subsystem;

/**
 * Slowly moves the intake arm from the deployed (down) position back toward stowed (up)
 * while the robot is shooting.
 *
 * <p>WAITS for the shooter to reach target RPM before starting the stow.
 * This prevents the arm from moving before the shooter is ready.
 *
 * <p>The arm only travels 80% of the way back (stops at 20% of its starting position).
 * This prevents the arm from slamming into the stow hard stop while pushing game pieces.
 *
 * <p>Runs in parallel with ShootV1Command and ShootAutoFeedCommand on the L1 button.
 * This command only requires IntakeDeploySubsystem — the shooter reference is read-only
 * so shooter and feeder commands are never interrupted.
 */
public class ShootStowArmCommand extends Command {

    // Fraction of the way to travel toward stow (0.0 = stay, 1.0 = full stow).
    private static final double kStowFraction = 0.8;

    private final IntakeDeploySubsystem m_arm;
    private final MotorGroup2Subsystem m_shooter;
    private boolean m_stowStarted = false;
    private double m_stowTarget = 0.0;

    public ShootStowArmCommand(IntakeDeploySubsystem arm, MotorGroup2Subsystem shooter) {
        m_arm = arm;
        m_shooter = shooter;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        m_stowStarted = false;
        m_stowTarget = 0.0;
    }

    @Override
    public void execute() {
        if (m_arm.isUp()) return;

        if (!m_stowStarted) {
            if (m_shooter.isOnTarget()) {
                m_stowStarted = true;
                // Calculate 80% of the way from current position toward 0.
                // e.g. at 20.0 rot → target = 20.0 * (1 - 0.8) = 4.0 rot
                double currentPos = m_arm.getPositionRotations();
                m_stowTarget = currentPos * (1.0 - kStowFraction);
                m_arm.setStallProtectionEnabled(false);
                m_arm.setMotionMagicCruiseVelocity(MotorConstants.kArmShootStowCruiseVelocity);
                m_arm.moveToRotations(m_stowTarget);
                System.out.println("[SHOOT-STOW] Shooter on target — arm stowing to "
                    + String.format("%.1f", m_stowTarget) + " rot (80% of " + String.format("%.1f", currentPos) + ")");
            } else {
                m_arm.holdPosition();
            }
        } else {
            m_arm.moveToRotations(m_stowTarget);
        }
    }

    @Override
    public boolean isFinished() {
        if (m_arm.isUp()) return true;
        if (m_stowStarted && Math.abs(m_arm.getPositionRotations() - m_stowTarget)
                < MotorConstants.kArmPositionToleranceRotations) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_arm.setStallProtectionEnabled(true);
        m_arm.setMotionMagicCruiseVelocity(MotorConstants.kArmCruiseVelocity);
        if (m_stowStarted) {
            // Hold at the partial stow position — don't push further
            m_arm.moveToRotations(m_stowTarget);
        }
        System.out.println("[SHOOT-STOW] Arm " + (interrupted ? "stopped mid-stow" : "reached partial stow"));
    }
}
