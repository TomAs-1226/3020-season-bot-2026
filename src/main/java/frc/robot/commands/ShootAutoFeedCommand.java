// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MotorGroup1Subsystem;
import frc.robot.subsystems.MotorGroup2Subsystem;

/**
 * ============================================================================
 *                    TEAM 3020 - SHOOT AUTO-FEED COMMAND
 * ============================================================================
 *
 * Automatically feeds game pieces once the shooter is at target RPM.
 *
 * REQUIRES: feeder subsystem ONLY (not shooter).
 *   The shooter is read-only — we check isOnTarget() without owning it.
 *   This means pressing Cross (manual feeder) interrupts THIS command but
 *   the shooter (ShootV1Command) keeps spinning independently.
 *
 * FEED LATCH:
 *   Once feeding starts, it NEVER stops until the button is released.
 *   This prevents RPM dips from game-piece load from interrupting the feed.
 *
 * SAFETY TIMEOUT:
 *   After 2.0s of spinning, starts feeding if RPM is at least 60% of target.
 *   Prevents the robot from never shooting because PID isn't perfectly tuned.
 *
 * BUTTON: L1 (hold) — bound independently alongside ShootV1Command.
 * ============================================================================
 */
public class ShootAutoFeedCommand extends Command {

    // After this many seconds, start feeding if at least MIN_FEED_FRACTION of target RPM.
    private static final double SPINUP_TIMEOUT_S  = 2.0;
    // Lowered from 0.60 to 0.40 — if shooter can't reach 60%, still allow feeding at 40%.
    private static final double MIN_FEED_FRACTION = 0.40;
    // Absolute hard timeout — start feeding regardless of RPM after this many seconds.
    // Prevents the robot from NEVER shooting in a match if PID is detuned.
    private static final double HARD_TIMEOUT_S    = 4.0;

    private final MotorGroup2Subsystem m_shooter;
    private final MotorGroup1Subsystem m_feeder;
    private final Timer                m_timer = new Timer();
    private boolean m_feedingLatched = false;

    public ShootAutoFeedCommand(MotorGroup2Subsystem shooter, MotorGroup1Subsystem feeder) {
        m_shooter = shooter;
        m_feeder  = feeder;
        // Only require feeder — shooter is read-only so it's never interrupted by us.
        addRequirements(feeder);
    }

    @Override
    public void initialize() {
        m_feedingLatched = false;
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        double avgRPM    = m_shooter.getAverageRPM();
        double targetRPM = m_shooter.getTargetRPM();

        boolean onTarget     = m_shooter.isOnTarget();
        boolean timeoutReady = m_timer.hasElapsed(SPINUP_TIMEOUT_S)
                            && avgRPM >= targetRPM * MIN_FEED_FRACTION;
        // Hard safety net: feed regardless after HARD_TIMEOUT_S
        boolean hardTimeout  = m_timer.hasElapsed(HARD_TIMEOUT_S);

        // Latch: once we start feeding, keep feeding even if RPM dips under load.
        if (onTarget || timeoutReady || hardTimeout) {
            m_feedingLatched = true;
        }

        if (m_feedingLatched) {
            m_feeder.runMotors();
        }
    }

    @Override
    public boolean isFinished() {
        return false; // runs until L1 released
    }

    @Override
    public void end(boolean interrupted) {
        m_feeder.stopMotors();
        m_feedingLatched = false;
    }
}
