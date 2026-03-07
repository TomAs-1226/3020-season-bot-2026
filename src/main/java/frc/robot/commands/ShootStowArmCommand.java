// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
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
 * <p>The arm targets slightly past encoder zero ({@code kShootStowOvershoot}) to
 * guarantee it physically seats against the stow hard stop even if encoder zero
 * is slightly off.  On completion the target is snapped back to 0.0 so
 * holdPosition() doesn't strain the gears.
 *
 * <p>Runs in parallel with ShootV1Command and ShootAutoFeedCommand on the L1 button.
 * This command only requires IntakeDeploySubsystem — the shooter reference is read-only
 * so shooter and feeder commands are never interrupted.
 *
 * <p>The arm stays stowed after the button is released — the default
 * holdPosition() command keeps it there.
 */
public class ShootStowArmCommand extends Command {

    // How far past encoder 0 the arm should target during shoot-stow.
    // Negative = past stow toward the hard stop.  Must stay within kArmReverseSoftLimit (-1.0).
    private static final double kShootStowOvershoot = -0.5;

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
                // Target slightly past zero so the arm physically seats at the hard stop
                m_arm.moveToRotations(kShootStowOvershoot);
                System.out.println("[SHOOT-STOW] Shooter on target — arm stowing to " + kShootStowOvershoot + " (overshoot mode)");
            } else {
                // Not ready yet — keep holding arm at current position
                m_arm.holdPosition();
            }
        } else {
            // Re-command every cycle so the arm keeps moving
            m_arm.moveToRotations(kShootStowOvershoot);
        }
    }

    @Override
    public boolean isFinished() {
        if (m_arm.isUp()) return true;
        // Finished when the arm reaches the overshoot target
        if (m_stowStarted && Math.abs(m_arm.getPositionRotations() - kShootStowOvershoot)
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
            // Snap arm to exactly zero so holdPosition() holds at home
            // without pushing against the hard stop.
            m_arm.moveToRotations(0.0);
        }
        System.out.println("[SHOOT-STOW] Arm " + (interrupted ? "stopped mid-stow" : "reached stow"));
    }
}
