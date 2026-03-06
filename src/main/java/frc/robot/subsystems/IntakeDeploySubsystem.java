// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ArmSetpoint;
import frc.robot.Constants.MotorConstants;

/**
 * ============================================================================
 *                    TEAM 3020 - INTAKE ARM SUBSYSTEM
 * ============================================================================
 *
 * Controls the intake arm using MotionMagicVoltage (trapezoidal profiling)
 * with gravity feedforward compensation.
 *
 * MOTOR: CAN ID 46 — Kraken X60 FOC, 1:36 gear ratio
 *
 * POSITIONS (motor shaft rotations):
 *   ArmSetpoint.STOWED   = 0     — arm fully UP   (zeroed at startup)
 *   ArmSetpoint.DEPLOYED = ~20   — arm fully DOWN  (tune via Shuffleboard)
 *
 * CONTROL EQUATION:
 *   volts = kS + kV * velocity + kG * cos(theta)
 *   - kS: static friction — smallest voltage that overcomes stiction
 *   - kV: velocity feedforward — proportional to target velocity (RPS)
 *   - kG: gravity gain — peak voltage at arm horizontal (max gravity torque)
 *   - cos(theta): scales gravity comp with actual arm angle automatically
 *   kS and kV live in Slot 0 (applied by MotionMagic internally).
 *   kG is computed each cycle and added via withFeedForward().
 *
 * TUNING WORKFLOW (Shuffleboard "Arm" tab — no redeployment needed):
 *   Step 1 — Run characterize command (Touchpad) to get kS and kV.
 *   Step 2 — Set kP to ~2.0, confirm arm reaches targets without oscillating.
 *   Step 3 — Hold arm at horizontal; increase "Arm kG" until arm holds position.
 *   Step 4 — Set "Arm Zero Angle (deg)" to match arm's physical stow angle.
 *   Step 5 — Tune "Arm Down Position (rot)" for the correct deployed height.
 *   All Shuffleboard values auto-save to Preferences and survive code deploys.
 *
 * STALL PROTECTION:
 *   If velocity stays below threshold for kDeployStallTimeThreshold seconds
 *   (after a startup grace period), the arm stops to prevent damage.
 *
 * @author Baichen Yu
 * ============================================================================
 */
public class IntakeDeploySubsystem extends SubsystemBase {

    /** The different states the arm can be in. */
    public enum DeployState {
        UP,           // Arm is at the stowed (up) position
        DOWN,         // Arm is at the deployed (down) position
        MOVING_UP,    // Arm is traveling to UP position
        MOVING_DOWN,  // Arm is traveling to DOWN position
        UNKNOWN       // Position unknown (e.g., after a mid-move stop)
    }

    // Motor — Kraken X60 FOC, CAN 46
    private final TalonFX m_motor;

    // MotionMagicVoltage: trapezoidal profiling using Slot 0 PID + MotionMagicConfigs.
    // FeedForward field is updated each cycle with the live kG*cos(theta) term.
    private final MotionMagicVoltage m_positionControl =
        new MotionMagicVoltage(0).withSlot(0).withEnableFOC(false);

    // DutyCycleOut: open-loop percent output.
    // Used ONLY by runRaw() (UnjamIntakeCommand, ArmCharacterizeCommand).
    private final DutyCycleOut m_dutyCycle = new DutyCycleOut(0).withEnableFOC(false);

    // Current state + target position
    private DeployState m_state = DeployState.UP; // Robot always starts with arm up
    private double m_targetPosition = 0.0;        // Motor shaft rotations; 0 = fully UP
    private boolean m_isMoving = false;

    // Stall detection — stops arm if velocity stays near zero while not at target
    private boolean m_stallDetected         = false;
    private double  m_stallStartTime        = 0.0;
    private double  m_moveStartTime         = 0.0;
    // Disabled by HomeArmCommand and ShootStowArmCommand where stalling is expected
    private boolean m_stallProtectionEnabled = true;

    // Set to true after the first successful homing sequence per power cycle.
    // Resets to false on construction (i.e. every power-on).
    // Used by RobotContainer to auto-home on the first deploy of each match.
    private boolean m_hasHomed = false;

    // Throttle Preferences saves — writing every 20ms floods flash and causes loop overrun
    private double m_lastPreferencesSaveTime = 0.0;

    // Cached tuning values — read from Shuffleboard entries each periodic cycle
    private double m_kG         = MotorConstants.kArmKg;
    private double m_zeroAngle  = MotorConstants.kArmZeroAngleDeg;

    // ===== SHUFFLEBOARD — "Arm" tab =====
    private final ShuffleboardTab m_tab = Shuffleboard.getTab("Arm");

    // ----- Tunable inputs -----
    /** Deployed (down) position in motor shaft rotations. Runtime-tunable. */
    private final GenericEntry m_entryDownPos =
        m_tab.add("Arm Down Position (rot)", MotorConstants.kArmDeployedRotations).getEntry();

    /**
     * kG — gravity feedforward gain.
     * Increase until the arm holds horizontal without drooping.
     * Formula: volts += kG * cos(theta)
     */
    private final GenericEntry m_entryKg =
        m_tab.add("Arm kG", MotorConstants.kArmKg).getEntry();

    /**
     * Arm angle (degrees above horizontal) when the encoder reads 0 (arm fully stowed).
     * Used to compute cos(theta) for gravity feedforward.
     * Typical value: ~90° if arm points straight up when stowed.
     */
    private final GenericEntry m_entryZeroAngle =
        m_tab.add("Arm Zero Angle (deg)", MotorConstants.kArmZeroAngleDeg).getEntry();

    // ----- Status / state readouts -----
    private final GenericEntry m_entryStatus   = m_tab.add("Arm Status",      "INIT").getEntry();
    private final GenericEntry m_entryState    = m_tab.add("Arm State",       "UP").getEntry();

    // ----- Position telemetry -----
    private final GenericEntry m_entryPosition = m_tab.add("Arm Position",    0.0).getEntry();
    private final GenericEntry m_entryTarget   = m_tab.add("Arm Target",      0.0).getEntry();
    private final GenericEntry m_entryError    = m_tab.add("Arm Error (rot)", 0.0).getEntry();
    private final GenericEntry m_entryAngleDeg = m_tab.add("Arm Angle (deg)", 0.0).getEntry();
    private final GenericEntry m_entryGravityFF= m_tab.add("Gravity FF (V)",  0.0).getEntry();

    // ----- Motor health -----
    private final GenericEntry m_entryAmps     = m_tab.add("Arm Amps",        0.0).getEntry();
    private final GenericEntry m_entryRPM      = m_tab.add("Arm RPM",         0.0).getEntry();
    private final GenericEntry m_entryMoving   = m_tab.add("Arm Moving",      false).getEntry();
    private final GenericEntry m_entryAtTarget = m_tab.add("Arm At Target",   false).getEntry();
    private final GenericEntry m_entryAlive    = m_tab.add("Arm Alive",       false).getEntry();
    private final GenericEntry m_entryStalled  = m_tab.add("Arm Stalled",     false).getEntry();

    // ----- Auto-tune control (used by ArmCharacterizeCommand) -----
    /**
     * Toggle to true in Shuffleboard to trigger the full auto-tune sequence.
     * Automatically reset to false when the sequence completes.
     * Steps before running:
     *   1. Set "Arm Zero Angle (deg)" to the arm's physical stow angle from horizontal.
     *   2. Set "Arm Down Position (rot)" to the approximate deployed position.
     *   3. Toggle "Request Auto Tune" to true.
     */
    private final GenericEntry m_entryRequestAutoTune =
        m_tab.add("Request Auto Tune", false)
             .withWidget(BuiltInWidgets.kToggleButton)
             .getEntry();

    /** Live status string updated by ArmCharacterizeCommand during the sequence. */
    private final GenericEntry m_entryAutoTuneStatus =
        m_tab.add("Auto Tune Status", "Not started").getEntry();

    /**
     * Creates the intake deploy subsystem.
     */
    public IntakeDeploySubsystem() {
        System.out.println("[ARM] Starting up (MotionMagicVoltage + kG gravity FF, Kraken X60, 1:36)...");

        m_motor = new TalonFX(MotorConstants.kMotorGroup4MotorID);

        // Load saved tuning values from Preferences → push to Shuffleboard
        m_entryDownPos.setDouble(
            Preferences.getDouble("Arm Down Position (rot)", MotorConstants.kArmDeployedRotations));
        m_entryKg.setDouble(
            Preferences.getDouble("Arm kG", MotorConstants.kArmKg));
        m_entryZeroAngle.setDouble(
            Preferences.getDouble("Arm Zero Angle (deg)", MotorConstants.kArmZeroAngleDeg));

        configureMotor();

        // Reset encoder — robot always starts with arm in the UP (stowed) position
        m_motor.setPosition(0.0);

        System.out.println("[ARM] Arm ready — CAN " + MotorConstants.kMotorGroup4MotorID
            + "  deployed target: " + MotorConstants.kArmDeployedRotations
            + " rot  kG: " + MotorConstants.kArmKg + " V");
    }

    /**
     * Configures the motor: current limits, brake mode, MotionMagic PID, and soft limits.
     * kG is NOT applied here — it is computed live each cycle via computeGravityFF().
     */
    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Current limiting
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimit = MotorConstants.kFalconCurrentLimit;
        currentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits = currentLimits;

        // Motor output — Brake mode holds the arm in place when not commanded
        MotorOutputConfigs motorOutput = new MotorOutputConfigs();
        motorOutput.Inverted = InvertedValue.Clockwise_Positive; // CW = positive = arm goes DOWN
        motorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput = motorOutput;

        // Slot 0 — kS and kV are handled here by MotionMagic.
        // kG is applied separately via withFeedForward() each cycle.
        Slot0Configs slot0 = new Slot0Configs();
        slot0.kP = MotorConstants.kArmKp;
        slot0.kI = MotorConstants.kArmKi;
        slot0.kD = MotorConstants.kArmKd;
        slot0.kV = MotorConstants.kArmKv;
        slot0.kS = MotorConstants.kArmKs;
        config.Slot0 = slot0;

        // MotionMagic profile — trapezoidal velocity/acceleration limits (motor shaft)
        MotionMagicConfigs motionMagic = new MotionMagicConfigs();
        motionMagic.MotionMagicCruiseVelocity = MotorConstants.kArmCruiseVelocity;
        motionMagic.MotionMagicAcceleration   = MotorConstants.kArmAcceleration;
        config.MotionMagic = motionMagic;

        // Software limits — absolute safety net; motor will never exceed these
        SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs();
        softLimits.ForwardSoftLimitThreshold = MotorConstants.kArmForwardSoftLimit;
        softLimits.ForwardSoftLimitEnable    = MotorConstants.kArmSoftLimitsEnable;
        softLimits.ReverseSoftLimitThreshold = MotorConstants.kArmReverseSoftLimit;
        softLimits.ReverseSoftLimitEnable    = MotorConstants.kArmSoftLimitsEnable;
        config.SoftwareLimitSwitch = softLimits;

        for (int i = 0; i < 3; i++) {
            if (m_motor.getConfigurator().apply(config).isOK()) break;
        }
    }

    // ========== GRAVITY FEEDFORWARD ==========

    /**
     * Returns the arm's physical angle from horizontal in degrees.
     *
     * <p>Uses {@code kArmZeroAngleDeg} as the arm's angle above horizontal when the encoder
     * reads 0 (arm fully stowed / UP). Going DOWN increases motor rotations, which decreases
     * the angle from horizontal (arm sweeps toward and past horizontal).
     *
     * <p>Formula:
     * <pre>
     *   armAngle = kArmZeroAngleDeg - (motorRotations / kArmGearRatio) * 360
     * </pre>
     */
    public double getArmAngleDeg() {
        double motorRot = m_motor.getPosition().getValueAsDouble();
        double mechanismRot = motorRot / MotorConstants.kArmGearRatio;
        return m_zeroAngle - mechanismRot * 360.0;
    }

    /**
     * Computes the live gravity feedforward voltage: {@code kG * cos(theta)}.
     *
     * <p>This term is added to every motor control request via
     * {@code withFeedForward()}, ensuring the arm always has the correct gravity
     * compensation regardless of whether it is moving or holding position.
     *
     * <p>Complete control equation applied to the motor:
     * <pre>
     *   volts = kS + kV * velocity + kP * error + kG * cos(theta)
     * </pre>
     * (kS and kV are in Slot 0; kG is this method's return value.)
     */
    private double computeGravityFF() {
        double angleRad = Math.toRadians(getArmAngleDeg());
        return m_kG * Math.cos(angleRad);
    }

    // ========== COMMANDS CALL THESE ==========

    /**
     * Move the arm to a named setpoint.
     *
     * <p>For {@link ArmSetpoint#DEPLOYED}, the runtime Shuffleboard value overrides
     * the compile-time default so field tuning does not require a redeploy.
     *
     * @param setpoint The target arm position.
     */
    public void moveToSetpoint(ArmSetpoint setpoint) {
        double target;
        if (setpoint == ArmSetpoint.DEPLOYED) {
            // Allow runtime override via Shuffleboard
            target = m_entryDownPos.getDouble(MotorConstants.kArmDeployedRotations);
        } else {
            target = setpoint.getMotorRotations();
        }

        // Hard clamp — target can never escape the software limit range.
        // The motor's hardware soft limits also enforce this, but clamping the
        // command prevents even issuing an out-of-range setpoint.
        target = Math.max(MotorConstants.kArmReverseSoftLimit + 0.1,
                 Math.min(MotorConstants.kArmForwardSoftLimit - 0.1, target));

        // If state is already correct, never re-command — even if encoder position is
        // outside tolerance.  State is set by periodic() arrival check OR the stall-at-bumper
        // block below; both mean the arm has physically arrived.  Without this guard the
        // stall-at-bumper block fires once, clears m_stallDetected, then the next call
        // re-commands the motor because alreadyThere was false → infinite re-command loop.
        if ((setpoint == ArmSetpoint.STOWED   && m_state == DeployState.UP) ||
                (setpoint == ArmSetpoint.DEPLOYED && m_state == DeployState.DOWN)) {
            return;
        }

        // If stall protection already stopped us while deploying, the arm is at the
        // physical bumper hard stop — that IS the deployed position.
        // Treat it as arrived so deploy commands stop re-commanding.
        if (m_stallDetected && setpoint == ArmSetpoint.DEPLOYED) {
            m_state        = DeployState.DOWN;
            m_isMoving     = false;
            m_targetPosition = target;
            m_stallDetected  = false;
            m_entryStatus.setString("DOWN (stall@bumper)");
            return;
        }

        m_targetPosition = target;
        m_motor.setControl(m_positionControl
            .withPosition(m_targetPosition)
            .withFeedForward(computeGravityFF()));

        m_state = (setpoint == ArmSetpoint.STOWED) ? DeployState.MOVING_UP : DeployState.MOVING_DOWN;
        m_isMoving = true;
        m_stallDetected = false;
        m_moveStartTime = Timer.getFPGATimestamp();
        m_entryStatus.setString(setpoint == ArmSetpoint.STOWED ? "GOING UP" : "GOING DOWN");
    }

    /**
     * Move arm to an arbitrary rotation target (motor shaft rotations).
     * Used by ArmCharacterizeCommand to position the arm for kG testing.
     * Target is automatically clamped to the soft-limit range.
     */
    public void moveToRotations(double rotations) {
        double clamped = Math.max(MotorConstants.kArmReverseSoftLimit + 0.1,
                         Math.min(MotorConstants.kArmForwardSoftLimit - 0.1, rotations));
        m_targetPosition = clamped;
        m_motor.setControl(m_positionControl
            .withPosition(m_targetPosition)
            .withFeedForward(computeGravityFF()));
        m_isMoving = true;
        m_stallDetected = false;
        m_moveStartTime = Timer.getFPGATimestamp();
        m_state = DeployState.UNKNOWN;
    }

    /**
     * Apply auto-tune results and make them take effect immediately without redeploying.
     *
     * <ul>
     *   <li>kS and kV are pushed to the motor's Slot 0 configuration live.</li>
     *   <li>kG is applied to the live feedforward cache and the Shuffleboard entry.</li>
     *   <li>All values are written to Preferences so they survive power cycles.</li>
     * </ul>
     *
     * Called by {@link frc.robot.commands.ArmCharacterizeCommand} at the end of its sequence.
     *
     * @param kS Static friction voltage (V)
     * @param kV Velocity feedforward (V·s/rotation, motor shaft)
     * @param kG Gravity feedforward gain (V at horizontal)
     */
    public void applyTuningResults(double kS, double kV, double kG) {
        // 1. Save to Preferences for persistence across power cycles
        Preferences.setDouble("Arm kS_tuned", kS);
        Preferences.setDouble("Arm kV_tuned", kV);
        Preferences.setDouble("Arm kG", kG);

        // 2. Apply kG immediately to the live feedforward cache and Shuffleboard entry
        m_kG = kG;
        m_entryKg.setDouble(kG);

        // 3. Push new kS/kV to motor Slot 0 without full reconfigure
        Slot0Configs slot0 = new Slot0Configs();
        slot0.kP = MotorConstants.kArmKp;
        slot0.kI = MotorConstants.kArmKi;
        slot0.kD = MotorConstants.kArmKd;
        slot0.kV = kV;
        slot0.kS = kS;
        for (int i = 0; i < 3; i++) {
            if (m_motor.getConfigurator().apply(slot0).isOK()) break;
        }

        System.out.println("[ARM] Tuning applied live — kS=" + String.format("%.4f", kS)
            + " kV=" + String.format("%.4f", kV)
            + " kG=" + String.format("%.4f", kG));
    }

    /**
     * Move arm to the deployed (down) position for floor intake.
     * Uses Shuffleboard value for runtime position tuning.
     */
    public void deployDown() {
        moveToSetpoint(ArmSetpoint.DEPLOYED);
    }

    /**
     * Move arm back to the stowed (up) position.
     */
    public void deployUp() {
        moveToSetpoint(ArmSetpoint.STOWED);
    }

    /**
     * Hold the arm at its current target position using closed-loop control with
     * live gravity feedforward. Called continuously by the default command.
     *
     * When the arm is at the deployed (DOWN) position, the motor holds at the
     * backed-off target (slightly away from the bumper hard stop). This prevents
     * the motor from pushing into the hard stop indefinitely due to kS + encoder noise.
     */
    public void holdPosition() {
        m_motor.setControl(m_positionControl
            .withPosition(m_targetPosition)
            .withFeedForward(computeGravityFF()));
    }

    /**
     * Freeze the arm at its current actual position.
     * Preserves arm position when a command is interrupted mid-move.
     */
    public void stop() {
        m_targetPosition = m_motor.getPosition().getValueAsDouble();
        m_motor.setControl(m_positionControl
            .withPosition(m_targetPosition)
            .withFeedForward(computeGravityFF()));
        m_isMoving = false;
        m_state = DeployState.UNKNOWN;
        m_entryStatus.setString("STOPPED");
    }

    /**
     * Re-calibrate the encoder to a known position.
     *
     * <p>Called by HomeArmCommand after the arm contacts the bumper hard stop.
     * Sets the motor's internal encoder to {@code rotations} so that all
     * setpoints are correct relative to that physical reference.
     *
     * @param rotations The known physical position (e.g. kArmDeployedRotations when at bumper).
     */
    public void setEncoderPosition(double rotations) {
        m_motor.setPosition(rotations);
        m_targetPosition = rotations;
        m_state = DeployState.DOWN;
        System.out.println("[ARM] Encoder re-zeroed — position set to " + rotations + " rot");
    }

    /**
     * Calibrate the deployed setpoint from the current bumper contact position.
     *
     * <p>Called by {@link frc.robot.commands.HomeArmCommand} once bumper stall is confirmed.
     * Reads the arm's ACTUAL encoder position (distance traveled from stow = 0) and saves
     * it as the new "Arm Down Position" in Shuffleboard and Preferences. The encoder is NOT
     * reset — stow=0 remains valid. All subsequent deploy commands go to this calibrated value.
     */
    public void calibrateDeployedPosition() {
        double bumperPos = m_motor.getPosition().getValueAsDouble();
        // Clamp to a physically sensible range (must be positive, below forward soft limit)
        double clamped = Math.max(1.0, Math.min(MotorConstants.kArmForwardSoftLimit - 0.1, bumperPos));
        m_entryDownPos.setDouble(clamped);
        Preferences.setDouble("Arm Down Position (rot)", clamped);
        m_targetPosition = clamped;
        m_state    = DeployState.DOWN;
        m_isMoving = false;
        System.out.println("[ARM] Deployed setpoint calibrated at "
            + String.format("%.2f", clamped) + " rot (stall-at-bumper homing)");
    }

    /**
     * Re-apply the active position setpoint with a freshly computed gravity feedforward.
     *
     * <p>Call in {@code execute()} of deploy commands so that {@code kG*cos(theta)} stays
     * accurate as the arm sweeps through its range. Does not reset any move-start timing
     * or stall-detection state.
     */
    public void refreshFeedForward() {
        if (m_isMoving) {
            m_motor.setControl(m_positionControl
                .withPosition(m_targetPosition)
                .withFeedForward(computeGravityFF()));
        }
    }

    /**
     * Emergency stop — sets output to zero immediately.
     * Use only if position hold is causing a problem.
     */
    public void forceStop() {
        m_motor.setControl(m_dutyCycle.withOutput(0));
        m_isMoving = false;
        m_state = DeployState.UNKNOWN;
        m_entryStatus.setString("FORCE STOPPED");
    }

    /**
     * Temporarily change the MotionMagic cruise velocity.
     * Call with {@code kArmShootStowCruiseVelocity} in ShootStowArmCommand.initialize()
     * and with {@code kArmCruiseVelocity} in ShootStowArmCommand.end() to restore.
     * Only 1 CAN write per call — safe to do in initialize/end.
     */
    public void setMotionMagicCruiseVelocity(double cruiseRPS) {
        MotionMagicConfigs mm = new MotionMagicConfigs();
        mm.MotionMagicCruiseVelocity = cruiseRPS;
        mm.MotionMagicAcceleration   = MotorConstants.kArmAcceleration;
        m_motor.getConfigurator().apply(mm);
    }

    /**
     * Run arm at a raw duty cycle, bypassing position control.
     * Used by UnjamIntakeCommand and ArmCharacterizeCommand only.
     * Positive = down, negative = up.
     */
    public void runRaw(double power) {
        m_motor.setControl(m_dutyCycle.withOutput(Math.max(-1.0, Math.min(1.0, power))));
        // Track position continuously so stop() holds wherever runRaw() left off
        m_targetPosition = m_motor.getPosition().getValueAsDouble();
        m_isMoving = Math.abs(power) > 0.01;
    }

    // ========== INFO GETTERS ==========

    /** Get the current arm state. */
    public DeployState getState() {
        return m_state;
    }

    /** Is the arm at the deployed (down) position? */
    public boolean isDown() {
        return m_state == DeployState.DOWN;
    }

    /** Is the arm at the stowed (up) position? */
    public boolean isUp() {
        return m_state == DeployState.UP;
    }

    /** Is the arm currently moving toward a target? */
    public boolean isMoving() {
        return m_isMoving;
    }

    /**
     * True when the arm is within position tolerance of the current target.
     * Used by DeployDownCommand and DeployUpCommand to detect completion.
     */
    public boolean isAtTarget() {
        return Math.abs(getPositionRotations() - m_targetPosition)
            < MotorConstants.kArmPositionToleranceRotations;
    }

    /** Motor shaft position in rotations (0 = stowed/up; positive = deployed/down). */
    public double getPositionRotations() {
        return m_motor.getPosition().getValueAsDouble();
    }

    /** Current amps drawn by the arm motor (supply side). */
    public double getCurrentAmps() {
        return m_motor.getSupplyCurrent().getValueAsDouble();
    }

    /** Current speed of the arm motor in RPM (absolute value). */
    public double getRPM() {
        return Math.abs(m_motor.getVelocity().getValueAsDouble() * 60.0);
    }

    // ========== AUTO-TUNE INTERFACE ==========

    /**
     * Returns true when the user has toggled "Request Auto Tune" in Shuffleboard.
     * Checked by RobotContainer to schedule ArmCharacterizeCommand.
     */
    public boolean isAutoTuneRequested() {
        return m_entryRequestAutoTune.getBoolean(false);
    }

    /**
     * True after the first successful homing sequence this power cycle.
     * Checked by RobotContainer to gate the auto-home-on-first-deploy logic.
     */
    public boolean hasHomed() {
        return m_hasHomed;
    }

    /** Called by HomeArmCommand when bumper contact is confirmed and encoder is zeroed. */
    public void setHomed(boolean homed) {
        m_hasHomed = homed;
    }

    /**
     * Enable or disable the built-in stall protection.
     * Disable before any command where stalling is intentional (homing, shoot-stow).
     * Always re-enable in the command's end() method.
     */
    public void setStallProtectionEnabled(boolean enabled) {
        m_stallProtectionEnabled = enabled;
    }

    /** Reset the Shuffleboard button after the auto-tune command finishes. */
    public void clearAutoTuneRequest() {
        m_entryRequestAutoTune.setBoolean(false);
    }

    /** Update the "Auto Tune Status" readout in Shuffleboard during the tune sequence. */
    public void setAutoTuneStatus(String status) {
        m_entryAutoTuneStatus.setString(status);
    }

    // ========== PERIODIC - RUNS EVERY 20MS ==========

    @Override
    public void periodic() {
        // Read live tuning values from Shuffleboard (allows zero-redeploy tuning)
        m_kG        = m_entryKg.getDouble(MotorConstants.kArmKg);
        m_zeroAngle = m_entryZeroAngle.getDouble(MotorConstants.kArmZeroAngleDeg);

        double position    = getPositionRotations();
        double angleDeg    = getArmAngleDeg();
        double gravityFF   = computeGravityFF();

        // Transition state when arm arrives at target
        if (m_state == DeployState.MOVING_DOWN && isAtTarget()) {
            m_state    = DeployState.DOWN;
            m_isMoving = false;
            System.out.println("[ARM] Reached DEPLOYED position");
            m_entryStatus.setString("DOWN");
        } else if (m_state == DeployState.MOVING_UP && isAtTarget()) {
            m_state    = DeployState.UP;
            m_isMoving = false;
            m_motor.setPosition(0.0); // Re-zero encoder at home
            m_targetPosition = 0.0;
            System.out.println("[ARM] Reached STOWED position");
            m_entryStatus.setString("UP");
        }

        // Stall protection — grace period ignores normal acceleration phase.
        // Uses BOTH velocity AND current to distinguish bumper stall from normal deceleration:
        //   At bumper:           RPM low (motor blocked)   AND amps high (fighting hard stop)
        //   Approaching target:  RPM low (MotionMagic braking)  BUT amps low (gentle stop)
        if (m_stallProtectionEnabled && m_isMoving && Timer.getFPGATimestamp() - m_moveStartTime > MotorConstants.kDeployStallStartupDelay) {
            boolean stalling = !isAtTarget()
                            && getRPM()          < MotorConstants.kDeployStallVelocityThreshold
                            && getCurrentAmps()  > MotorConstants.kDeployStallCurrentThreshold;
            if (stalling) {
                if (!m_stallDetected) {
                    m_stallDetected  = true;
                    m_stallStartTime = Timer.getFPGATimestamp();
                } else if (Timer.getFPGATimestamp() - m_stallStartTime >= MotorConstants.kDeployStallTimeThreshold) {
                    DeployState stallState = m_state;
                    if (stallState == DeployState.MOVING_DOWN) {
                        // At deploy bumper: back off a tiny amount so the motor isn't
                        // fighting the hard stop, then hold THAT position.
                        double currentPos = m_motor.getPosition().getValueAsDouble();
                        m_targetPosition = currentPos - MotorConstants.kArmStallBackoffRotations;
                        m_motor.setControl(m_positionControl
                            .withPosition(m_targetPosition)
                            .withFeedForward(computeGravityFF()));
                        m_isMoving = false;
                        m_stallDetected = false;
                        m_state = DeployState.DOWN;
                        m_entryStatus.setString("STALL STOP (DOWN) backed off to " + String.format("%.1f", m_targetPosition));
                    } else if (stallState == DeployState.MOVING_UP) {
                        stop();
                        m_state = DeployState.UP;
                        m_entryStatus.setString("STALL STOP (UP)");
                    } else {
                        stop();
                        m_entryStatus.setString("STALL STOP");
                    }
                    System.out.println("[ARM] STALL DETECTED — stopped at " + stallState);
                }
            } else {
                // Current is low = not fighting anything = safe to reset stall timer.
                // The current check prevents deceleration toward target from false-triggering.
                m_stallDetected = false;
            }
        }

        // Movement timeout — hard safety net if stall detection fails.
        // If the arm has been moving for longer than the timeout, force-stop it.
        // This prevents the arm from pushing into a physical stop indefinitely
        // (e.g. if stall current is below the detection threshold).
        if (m_stallProtectionEnabled && m_isMoving
                && Timer.getFPGATimestamp() - m_moveStartTime > MotorConstants.kDeployMoveTimeoutSeconds) {
            DeployState timeoutState = m_state;
            if (timeoutState == DeployState.MOVING_DOWN) {
                // Back off from the bumper like stall detection does
                double currentPos = m_motor.getPosition().getValueAsDouble();
                m_targetPosition = currentPos - MotorConstants.kArmStallBackoffRotations;
                m_motor.setControl(m_positionControl
                    .withPosition(m_targetPosition)
                    .withFeedForward(computeGravityFF()));
                m_isMoving = false;
                m_stallDetected = false;
                m_state = DeployState.DOWN;
                m_entryStatus.setString("TIMEOUT STOP (DOWN)");
            } else if (timeoutState == DeployState.MOVING_UP) {
                stop();
                m_state = DeployState.UP;
                m_entryStatus.setString("TIMEOUT STOP (UP)");
            } else {
                stop();
                m_entryStatus.setString("TIMEOUT STOP");
            }
            System.out.println("[ARM] MOVE TIMEOUT — force-stopped at " + timeoutState
                + " after " + MotorConstants.kDeployMoveTimeoutSeconds + "s");
        }

        // Auto-save tuning inputs to Preferences — throttled to every 5s to prevent loop overrun
        double nowSave = Timer.getFPGATimestamp();
        if (nowSave - m_lastPreferencesSaveTime > 5.0) {
            m_lastPreferencesSaveTime = nowSave;
            Preferences.setDouble("Arm Down Position (rot)",
                m_entryDownPos.getDouble(MotorConstants.kArmDeployedRotations));
            Preferences.setDouble("Arm kG",   m_kG);
            Preferences.setDouble("Arm Zero Angle (deg)", m_zeroAngle);
        }

        // Telemetry → Shuffleboard "Arm" tab
        m_entryState.setString(m_state.toString());
        m_entryPosition.setDouble(position);
        m_entryTarget.setDouble(m_targetPosition);
        m_entryError.setDouble(m_targetPosition - position);
        m_entryAngleDeg.setDouble(angleDeg);
        m_entryGravityFF.setDouble(gravityFF);
        m_entryAmps.setDouble(getCurrentAmps());
        m_entryRPM.setDouble(getRPM());
        m_entryMoving.setBoolean(m_isMoving);
        m_entryAtTarget.setBoolean(isAtTarget());
        m_entryAlive.setBoolean(m_motor.isAlive());
        m_entryStalled.setBoolean(m_stallDetected);
    }
}
