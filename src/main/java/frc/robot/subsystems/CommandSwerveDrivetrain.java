// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
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
 * PathPlanner AutoBuilder is configured here for autonomous path following.
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

    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
        System.out.println("[DRIVE] Swerve drivetrain initialized with PathPlanner AutoBuilder");
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

    /** Returns the current robot pose from CTRE odometry. */
    public Pose2d getPose() {
        return getState().Pose;
    }

    /** Resets the robot's pose estimate (used by PathPlanner at auto start). */
    public void resetOdometry(Pose2d pose) {
        super.resetPose(pose);
    }

    /** Returns robot-relative chassis speeds from CTRE odometry. */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return getState().Speeds;
    }

    /** Configures PathPlanner AutoBuilder for autonomous path following. */
    private void configureAutoBuilder() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            System.err.println("[DRIVE] ERROR: Failed to load PathPlanner RobotConfig from GUI settings!");
            e.printStackTrace();
            return;
        }

        AutoBuilder.configure(
            this::getPose,
            this::resetOdometry,
            this::getRobotRelativeSpeeds,
            (ChassisSpeeds speeds, DriveFeedforwards feedforwards) -> {
                setControl(m_pathApplyRobotSpeeds.withSpeeds(speeds));
            },
            new PPHolonomicDriveController(
                new com.pathplanner.lib.config.PIDConstants(5.0, 0.0, 0.0),  // translation PID
                new com.pathplanner.lib.config.PIDConstants(5.0, 0.0, 0.0)   // rotation PID
            ),
            config,
            () -> {
                // Flip path for red alliance (PathPlanner paths are drawn for blue)
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );

        System.out.println("[DRIVE] PathPlanner AutoBuilder configured");
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
