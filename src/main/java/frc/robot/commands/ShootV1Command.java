// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MotorGroup2Subsystem;

/**
 * ============================================================================
 *                    TEAM 3020 - SHOOT V1 COMMAND
 * ============================================================================
 *
 * HOW IT WORKS:
 *   1. Immediately starts spinning up the flywheel to target RPM.
 *   2. Re-applies velocity EVERY cycle so the shooter NEVER dies from CAN
 *      resets, voltage sags, or any other transient event.
 *   3. Keeps running until the button is released.
 *   4. On release: flywheel drops to idle (or stops if idle disabled).
 *
 * IMPORTANT: This command ONLY requires the shooter subsystem.
 *   Feeding is handled by a SEPARATE command bound independently to L1
 *   in RobotContainer. This ensures that pressing Cross (manual feeder),
 *   L2 (outtake), or any other feeder command NEVER interrupts the shooter.
 *
 * STATUS is written to the "Shooter Status" field in Shuffleboard "Shooter" tab.
 *
 * BUTTON: L1 (hold to shoot)
 * ============================================================================
 */
public class ShootV1Command extends Command {

    private final MotorGroup2Subsystem m_shooter;
    private final Timer                m_timer = new Timer();

    public ShootV1Command(MotorGroup2Subsystem shooter) {
        m_shooter = shooter;
        // ONLY require shooter — feeder is handled independently so manual
        // feeder commands (Cross, L2) never kill the shooter.
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
        m_shooter.runForward();
        m_shooter.setShooterStatus("SPINNING UP...");
    }

    @Override
    public void execute() {
        // Re-command velocity EVERY cycle — this is the RPM enforcement.
        // Even if CAN resets, voltage sags, or anything else happens,
        // the shooter will recover within one 20ms cycle.
        m_shooter.runForward();

        double avgRPM    = m_shooter.getAverageRPM();
        double targetRPM = m_shooter.getTargetRPM();
        boolean onTarget = m_shooter.isOnTarget();

        if (onTarget) {
            m_shooter.setShooterStatus("ON TARGET — " + (int) avgRPM + " RPM");
        } else {
            m_shooter.setShooterStatus(
                "SPINNING... " + (int) avgRPM + " / " + (int) targetRPM + " RPM");
        }
    }

    @Override
    public boolean isFinished() {
        return false; // runs until button released
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stopMotors();
        // stopMotors() sets status to "IDLE @ X RPM" or "STOPPED" automatically
    }
}
