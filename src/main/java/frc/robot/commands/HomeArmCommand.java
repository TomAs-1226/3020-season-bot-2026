// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeDeploySubsystem;

/**
 * Homes the intake arm by driving it slowly DOWN until it contacts the bumper
 * (physical hard stop), then sets the encoder to {@code kArmDeployedRotations}
 * at that point.
 *
 * <p>After this command completes, the encoder knows the arm is at the deployed
 * position — stow and all other setpoints are automatically correct.
 *
 * <p>Run this whenever arm position may be uncertain (e.g. arm was moved by hand
 * before power-on, or arm behaviour looks offset during a match).
 *
 * <p>Bind: Create button (onTrue)
 *
 * <p>Approach borrowed from Tagalong's {@code PivotEncoderlessZeroCmd} used by
 * Team 5805. No limit switch or absolute encoder needed — bumper contact is
 * the physical reference.
 */
public class HomeArmCommand extends Command {

    private final IntakeDeploySubsystem m_arm;

    // How fast to drive down while searching for the bumper.
    // 22 % is fast enough to home quickly mid-match (~1-2 s) while still gentle at contact.
    private static final double SEEK_DUTY_CYCLE   = 0.22;

    // How long after the move starts before stall detection activates.
    // Shorter grace at higher duty cycle — motor accelerates faster.
    private static final double GRACE_S           = 0.3;

    // Motor shaft RPM below which the arm is considered stalled.
    // 150 RPM confirmed for 0.15 s is sufficient to detect bumper contact reliably.
    // A supply-current check was previously required but removed: at 22 % duty cycle
    // the supply current at stall is too low and variable to be a reliable gate,
    // causing the first homing attempt to time out and zero at the wrong position.
    private static final double STALL_VEL_RPM     = 150.0;

    // Both conditions must hold for this long before zeroing.
    private static final double STALL_CONFIRM_S   = 0.15;

    // Give up after this long even if no stall is detected (best-effort zero).
    private static final double TIMEOUT_S         = 5.0;

    private double  m_startTime;
    private boolean m_stallSeen;
    private double  m_stallStart;
    private boolean m_done;

    public HomeArmCommand(IntakeDeploySubsystem arm) {
        m_arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        m_startTime = Timer.getFPGATimestamp();
        m_stallSeen = false;
        m_stallStart = 0;
        m_done = false;

        // Stall IS the detection mechanism — disable subsystem stall protection
        // so it doesn't call stop() before we detect bumper contact.
        m_arm.setStallProtectionEnabled(false);

        m_arm.setAutoTuneStatus("HOMING — driving to bumpers...");
        System.out.println("[HOME ARM] Homing sequence started");
    }

    @Override
    public void execute() {
        double now     = Timer.getFPGATimestamp();
        double elapsed = now - m_startTime;

        if (elapsed > TIMEOUT_S) {
            // Arm never hit bumper — zero wherever it ended up (best-effort)
            zeroHere("TIMEOUT — zeroed at current position");
            return;
        }

        m_arm.runRaw(SEEK_DUTY_CYCLE); // positive = down toward bumper

        if (elapsed < GRACE_S) return; // let motor accelerate before checking

        boolean stalling = m_arm.getRPM() < STALL_VEL_RPM;
        if (stalling) {
            if (!m_stallSeen) {
                m_stallSeen  = true;
                m_stallStart = now;
            } else if (now - m_stallStart >= STALL_CONFIRM_S) {
                zeroHere("Bumper contact — encoder zeroed at deployed");
            }
        } else {
            m_stallSeen = false; // not a confirmed stall — keep looking
        }
    }

    private void zeroHere(String reason) {
        // Record the ACTUAL encoder position at the bumper as the new deploy setpoint.
        // This keeps stow=0 correct and calibrates the deployed position to the real bumper
        // location instead of an arbitrary constant.
        m_arm.calibrateDeployedPosition();
        m_arm.setHomed(true); // gates the first-deploy auto-home in RobotContainer
        m_done = true;
        System.out.println("[HOME ARM] " + reason);
        m_arm.setAutoTuneStatus("HOMED — " + reason);
    }

    @Override
    public void end(boolean interrupted) {
        m_arm.setStallProtectionEnabled(true);
        if (interrupted && !m_done) {
            System.out.println("[HOME ARM] Interrupted before contact");
            m_arm.setAutoTuneStatus("HOMING interrupted");
        }
        // Leave arm at bumper position — operator can press D-pad Up to stow.
        m_arm.stop();
    }

    @Override
    public boolean isFinished() {
        return m_done;
    }
}
