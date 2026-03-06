// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * ============================================================================
 *                    TEAM 3020 - SWERVE DRIVETRAIN
 * ============================================================================
 *
 * CTRE Phoenix 6 swerve drivetrain wrapped as a WPILib Subsystem.
 * Uses field-centric control - push the stick forward and the robot drives
 * away from you no matter which way it's facing.
 *
 * DRIVING (PS5 Controller port 0):
 *   Left Stick  = Translation (forward/back/strafe)
 *   Right Stick X = Rotation
 *   L3 (push left stick) = X-lock wheels (brake)
 *   Options     = Reset field orientation (re-zero heading)
 *
 * CAN IDs (all on RIO bus):
 *   FL: drive=12, steer=11, encoder=13
 *   FR: drive=21, steer=20, encoder=22
 *   BL: drive=15, steer=14, encoder=16
 *   BR: drive=18, steer=17, encoder=19
 *   Pigeon2: 23
 * ============================================================================
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

    private static final double kSimLoopPeriod = 0.005; // 5 ms sim update rate
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        System.out.println("[DRIVE] Swerve drivetrain initialized");
    }

    /**
     * Returns a Command that applies the given swerve request every loop.
     * Use this to set the default drive command.
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Resets the field-centric heading so the robot's current direction is
     * treated as "field forward". Press Options to call this mid-match if
     * the gyro drifts or the robot gets turned around.
     */
    public void seedFieldCentric() {
        getPigeon2().setYaw(0);
    }

    @Override
    public void periodic() {
        // CTRE's SwerveDrivetrain handles odometry updates internally at high frequency.
        // Nothing extra needed here.
    }

    /** Starts the simulation update loop at 200 Hz. */
    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}
