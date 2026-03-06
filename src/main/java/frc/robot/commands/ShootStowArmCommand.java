// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ArmSetpoint;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.IntakeDeploySubsystem;
import frc.robot.subsystems.MotorGroup2Subsystem;

/**
 * Slowly moves the intake arm from the deployed (down) position back to stowed (up)
 * while the robot is shooting.
 *
 * <p>WAITS for the shooter to reach target RPM before starting the stow.
 * This prevents the arm from moving before the shooter is ready.
 *
 * <p>As the arm rises it pushes game pieces toward the shooter / indexer, which causes
 * resistance.  Stall protection is disabled for this command so the MotionMagic
 * closed-loop controller can push through that resistance and still reach the exact
 * stowed encoder position.
 *
 * <p>Runs in parallel with ShootV1Command and ShootAutoFeedCommand on the L1 button.
 * This command only requires IntakeDeploySubsystem — the shooter reference is read-only
 * so shooter and feeder commands are never interrupted.
 *
 * <p>The arm stays stowed after the button is released — the default
 * holdPosition() command keeps it there.
 */
public class ShootStowArmCommand extends Command {

    private final IntakeDeploySubsystem m_arm;
    private final MotorGroup2Subsystem m_shooter;
    private boolean m_stowStarted = false;

    public ShootStowArmCommand(IntakeDeploySubsystem arm, MotorGroup2Subsystem shooter) {
        m_arm = arm;
        m_shooter = shooter;
        // Only requires arm — shooter is read-only (checked for isOnTarget).
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        m_stowStarted = false;
        if (m_arm.isUp()) {
            // Already stowed — nothing to do, let isFinished() return true immediately.
            return;
        }
    }

    @Override
    public void execute() {
        if (m_arm.isUp()) return;

        if (!m_stowStarted) {
            // Wait for shooter to be on target before starting arm stow.
            // NO TIMEOUT — if shooter never reaches target, arm stays put.
            // This prevents the old bug where the arm moved before the shooter was ready.
            if (m_shooter.isOnTarget()) {
                m_stowStarted = true;
                // Resistance from balls is expected — disable stall protection
                m_arm.setStallProtectionEnabled(false);
                // Slow cruise so the arm gently pushes balls into the indexer
                m_arm.setMotionMagicCruiseVelocity(MotorConstants.kArmShootStowCruiseVelocity);
                m_arm.moveToSetpoint(ArmSetpoint.STOWED);
                System.out.println("[SHOOT-STOW] Shooter on target — arm stowing (slow push mode)");
            } else {
                // Not ready yet — keep holding arm at current position
                m_arm.holdPosition();
            }
        } else {
            // Re-command every cycle so the arm keeps moving
            m_arm.moveToSetpoint(ArmSetpoint.STOWED);
        }
    }

    @Override
    public boolean isFinished() {
        return m_arm.isUp() || (m_stowStarted && m_arm.isAtTarget());
    }

    @Override
    public void end(boolean interrupted) {
        m_arm.setStallProtectionEnabled(true);
        m_arm.setMotionMagicCruiseVelocity(MotorConstants.kArmCruiseVelocity);
        if (interrupted && m_stowStarted) {
            m_arm.stop();
        }
        System.out.println("[SHOOT-STOW] Arm " + (interrupted ? "stopped mid-stow" : "reached stow"));
    }
}
