// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.IntakeDeploySubsystem;

/**
 * ============================================================================
 *             TEAM 3020 — ARM FULL AUTO-CHARACTERIZE COMMAND
 * ============================================================================
 *
 * Fully automatic sequence.  HOLD Touchpad OR toggle "Request Auto Tune" in
 * Shuffleboard "Arm" tab.  The arm WILL move — ensure clearance first.
 *
 * Uses the motor's INTERNAL encoder (TalonFX rotor sensor) converted to
 * physical arm angle via kArmGearRatio and kArmZeroAngleDeg.
 *
 * ── BEFORE RUNNING ──────────────────────────────────────────────────────────
 *   In Shuffleboard "Arm" tab:
 *     "Arm Zero Angle (deg)" — arm's physical angle above horizontal when
 *                               encoder reads 0 (arm fully stowed/up).
 *                               Measure from robot geometry (typically 85–95°).
 *     "Arm Down Position (rot)" — approximate deployed rotation count.
 *
 * ── SEQUENCE (6 phases) ─────────────────────────────────────────────────────
 *
 *  Phase 1 — KS_RAMP
 *    Ramp voltage downward until arm first moves.
 *    → kS = friction threshold voltage.
 *
 *  Phase 2 — FIND_BOTTOM_STOP
 *    Continue slow downward voltage until arm stalls at physical bottom stop.
 *    → Records bottom arm angle A1 = getArmAngleDeg().
 *
 *  Phase 3 — BOTTOM_LIFTOFF
 *    From the bottom stop, ramp UPWARD voltage until arm just lifts off.
 *    → V1 = kS + kG·cos(A1)
 *
 *  Phase 4 — KV_TEST
 *    Apply 2 V / 3 V / 4 V downward for 0.5 s each; fit velocity–voltage line.
 *    → kV = (V − kS) / velocity
 *
 *  Phase 5 — FIND_TOP_STOP
 *    Drive arm upward until stall at physical top stop.
 *    → Records top arm angle A2 = getArmAngleDeg().
 *
 *  Phase 6 — TOP_HOLD_FIND
 *    Start with a moderate upward holding voltage; slowly decrease until arm
 *    just begins to fall.
 *    → V2 = kG·cos(A2) − kS
 *
 *  Phase 7 — DONE
 *    Solve:
 *       kG = (V1 + V2) / (cos(A1) + cos(A2))
 *       kS = V1 − kG·cos(A1)          (refines/cross-checks kS ramp result)
 *    Call applyTuningResults(kS, kV, kG) — values are live immediately AND
 *    saved to Preferences (survive power cycles).
 *    Print final values to console and SmartDashboard.
 *
 * ── READING RESULTS ─────────────────────────────────────────────────────────
 *   SmartDashboard: "Arm kS", "Arm kV", "Arm kG", "Arm Tune Phase"
 *   Console: full summary at end.
 *   Constants.java: update kArmKs / kArmKv / kArmKg with the printed values.
 *
 * ============================================================================
 */
public class ArmCharacterizeCommand extends Command {

    // ── Phase 1: kS ramp ─────────────────────────────────────────────────────
    private static final double KS_RAMP_RATE_V_PER_S    = 0.3;   // V/s downward
    private static final double KS_MOTION_THRESHOLD_RPS  = 0.05;  // RPS = arm started moving

    // ── Phase 2 & 5: finding hard stops ──────────────────────────────────────
    private static final double STOP_SEEK_DUTY          = 0.15;  // low duty to slowly reach stop
    private static final double STOP_GRACE_S            = 0.6;   // ignore stall during this window
    private static final double STOP_STALL_VEL_RPS      = 0.04;  // RPS — below this = stalled
    private static final double STOP_STALL_CONFIRM_S    = 0.20;  // must stall this long to confirm
    private static final double STOP_TIMEOUT_S          = 6.0;   // give up after this

    // ── Phase 3: bottom liftoff ───────────────────────────────────────────────
    private static final double LIFTOFF_RAMP_V_PER_S    = 0.12;  // V/s upward ramp
    private static final double LIFTOFF_DETECT_RPS      = 0.06;  // RPS = arm lifted
    private static final double LIFTOFF_MAX_V           = 10.0;  // safety ceiling

    // ── Phase 4: kV tests ─────────────────────────────────────────────────────
    private static final double   KV_TEST_DURATION_S    = 0.5;
    private static final double[] KV_TEST_VOLTAGES      = {2.0, 3.0, 4.0};

    // ── Phase 6: top hold find ────────────────────────────────────────────────
    private static final double TOP_START_HOLD_V        = 4.0;   // start holding voltage (up, V)
    private static final double TOP_DECREASE_V_PER_S    = 0.08;  // decrease rate V/s
    private static final double TOP_FALL_DETECT_RPS     = 0.05;  // RPS = arm fell = previous V was min
    private static final double TOP_MIN_HOLD_V          = 0.0;   // minimum before giving up

    // ── Minimum cos(θ) for reliable kG ───────────────────────────────────────
    private static final double MIN_COS_THETA           = 0.12;

    // ── Phases ────────────────────────────────────────────────────────────────
    private enum Phase {
        KS_RAMP,
        FIND_BOTTOM_STOP,
        BOTTOM_LIFTOFF,
        KV_TEST,
        FIND_TOP_STOP,
        TOP_HOLD_FIND,
        DONE
    }

    private final IntakeDeploySubsystem m_arm;

    // State
    private Phase  m_phase;
    private double m_phaseStart;   // timestamp when current phase began
    private double m_voltage;      // current ramp/test voltage (magnitude, always positive)

    // Stop-finding state
    private double  m_stallStart;
    private boolean m_stalledConfirmed;

    // kS ramp state
    private double m_resultKs = 0.0;

    // kV test state
    private int    m_kvIndex;
    private double m_kvPhaseStart;
    private double m_kvVoltageAccum;
    private double m_kvVelAccum;
    private int    m_kvSamples;
    private double m_sumV;
    private double m_sumVel;
    private double m_resultKv = 0.0;

    // Bottom / top angles (degrees from horizontal), recorded after each stop
    private double m_angleBottom = 0.0;
    private double m_angleTop    = 0.0;

    // Liftoff / hold voltages
    private double m_V1 = 0.0;  // upward liftoff voltage at bottom
    private double m_V2 = 0.0;  // minimum upward holding voltage at top

    // Previous voltage for TOP_HOLD_FIND (V2 is the last value BEFORE arm moved)
    private double m_prevHoldV  = TOP_START_HOLD_V;

    // Results
    private double m_resultKg = 0.0;

    public ArmCharacterizeCommand(IntakeDeploySubsystem arm) {
        m_arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        m_phase    = Phase.KS_RAMP;
        m_phaseStart = Timer.getFPGATimestamp();
        m_voltage  = 0.0;
        m_resultKs = 0.0;
        m_resultKv = 0.0;
        m_resultKg = 0.0;
        m_V1 = m_V2 = 0.0;
        m_sumV = m_sumVel = 0.0;
        m_kvIndex = 0;
        m_stalledConfirmed = false;

        status("kS ramp starting...");
        SmartDashboard.putString("Arm Tune Phase", "1/6  kS RAMP");
        SmartDashboard.putNumber("Arm kS", 0);
        SmartDashboard.putNumber("Arm kV", 0);
        SmartDashboard.putNumber("Arm kG", 0);
        System.out.println("[ARM TUNE] ===== Auto-characterization started =====");
        System.out.println("[ARM TUNE] kArmZeroAngleDeg = " + m_arm.getArmAngleDeg()
                + "° (current, arm should be stowed)");
    }

    @Override
    public void execute() {
        double now   = Timer.getFPGATimestamp();
        double vel   = velRPS();

        switch (m_phase) {

            // ── 1: kS ramp ───────────────────────────────────────────────────
            case KS_RAMP: {
                double elapsed = now - m_phaseStart;
                m_voltage = KS_RAMP_RATE_V_PER_S * elapsed;
                applyVolts(m_voltage);  // positive = downward

                if (vel > KS_MOTION_THRESHOLD_RPS) {
                    m_resultKs = m_voltage;
                    SmartDashboard.putNumber("Arm kS", m_resultKs);
                    System.out.println("[ARM TUNE] Phase 1 done — kS = "
                            + fmt(m_resultKs) + " V");
                    enterPhase(Phase.FIND_BOTTOM_STOP);
                }
                break;
            }

            // ── 2: find bottom hard stop ─────────────────────────────────────
            case FIND_BOTTOM_STOP: {
                applyVolts(12.0 * STOP_SEEK_DUTY); // slow downward
                status("2/6  Seeking bottom hard stop...");

                if (checkStall(now)) {
                    m_angleBottom = m_arm.getArmAngleDeg();
                    System.out.println("[ARM TUNE] Phase 2 done — bottom stop at "
                            + fmt(m_arm.getPositionRotations()) + " rot  angle="
                            + fmt(m_angleBottom) + "°");
                    m_voltage = 0.0;
                    enterPhase(Phase.BOTTOM_LIFTOFF);
                }
                break;
            }

            // ── 3: ramp upward until bottom liftoff ──────────────────────────
            case BOTTOM_LIFTOFF: {
                m_voltage += LIFTOFF_RAMP_V_PER_S * 0.02; // integrate ~20 ms period
                applyVolts(-m_voltage); // negative = upward

                status("3/6  Liftoff ramp: " + fmt(m_voltage) + " V up");

                if (vel > LIFTOFF_DETECT_RPS) {
                    m_V1 = m_voltage;
                    System.out.println("[ARM TUNE] Phase 3 done — V1 (liftoff) = "
                            + fmt(m_V1) + " V");
                    enterPhase(Phase.KV_TEST);
                    startKvTest(now);
                } else if (m_voltage >= LIFTOFF_MAX_V) {
                    System.out.println("[ARM TUNE] WARNING: liftoff max voltage reached — "
                            + "kG may be very large. Check mechanism.");
                    m_V1 = m_voltage;
                    enterPhase(Phase.KV_TEST);
                    startKvTest(now);
                }
                break;
            }

            // ── 4: kV velocity tests ─────────────────────────────────────────
            case KV_TEST: {
                double testV       = KV_TEST_VOLTAGES[m_kvIndex];
                double phaseElapsed = now - m_kvPhaseStart;
                applyVolts(testV); // positive = downward

                status("4/6  kV test " + (m_kvIndex + 1) + "/" + KV_TEST_VOLTAGES.length
                        + " @ " + testV + " V");

                // Accumulate steady-state samples in the second half
                if (phaseElapsed > KV_TEST_DURATION_S / 2.0) {
                    m_kvVoltageAccum += testV;
                    m_kvVelAccum     += vel;
                    m_kvSamples++;
                }

                if (phaseElapsed >= KV_TEST_DURATION_S) {
                    if (m_kvSamples > 0) {
                        m_sumV   += m_kvVoltageAccum / m_kvSamples;
                        m_sumVel += m_kvVelAccum     / m_kvSamples;
                        System.out.println("[ARM TUNE]   kV sample: "
                                + fmt(m_kvVoltageAccum / m_kvSamples) + " V → "
                                + fmt(m_kvVelAccum / m_kvSamples) + " RPS");
                    }
                    m_kvIndex++;
                    if (m_kvIndex >= KV_TEST_VOLTAGES.length) {
                        computeKv();
                        enterPhase(Phase.FIND_TOP_STOP);
                    } else {
                        startKvTest(now);
                    }
                }
                break;
            }

            // ── 5: find top hard stop ────────────────────────────────────────
            case FIND_TOP_STOP: {
                applyVolts(-12.0 * STOP_SEEK_DUTY); // slow upward
                status("5/6  Seeking top hard stop...");

                if (checkStall(now)) {
                    m_angleTop = m_arm.getArmAngleDeg();
                    System.out.println("[ARM TUNE] Phase 5 done — top stop at "
                            + fmt(m_arm.getPositionRotations()) + " rot  angle="
                            + fmt(m_angleTop) + "°");
                    m_voltage      = TOP_START_HOLD_V;
                    m_prevHoldV    = TOP_START_HOLD_V;
                    enterPhase(Phase.TOP_HOLD_FIND);
                }
                break;
            }

            // ── 6: decrease holding voltage to find minimum V2 ───────────────
            case TOP_HOLD_FIND: {
                applyVolts(-m_voltage); // upward (negative)
                status("6/6  Top hold find: " + fmt(m_voltage) + " V up");

                if (vel > TOP_FALL_DETECT_RPS) {
                    // Arm just started to fall — previous voltage was the minimum hold
                    m_V2 = m_prevHoldV;
                    System.out.println("[ARM TUNE] Phase 6 done — V2 (min top hold) = "
                            + fmt(m_V2) + " V");
                    finishCharacterization();
                } else if (m_voltage <= TOP_MIN_HOLD_V) {
                    // Ran out of voltage to decrease — arm holds even at 0V (near vertical)
                    m_V2 = 0.0;
                    System.out.println("[ARM TUNE] Phase 6 done — V2 = 0 (arm holds at top by gravity/friction)");
                    finishCharacterization();
                } else {
                    // Still holding — record previous voltage, decrease for next cycle
                    m_prevHoldV = m_voltage;
                    m_voltage  -= TOP_DECREASE_V_PER_S * 0.02; // ~20 ms
                    m_voltage   = Math.max(TOP_MIN_HOLD_V, m_voltage);
                }
                break;
            }

            case DONE:
                applyVolts(0);
                break;
        }

        SmartDashboard.putNumber("Arm Tune Voltage", m_voltage);
    }

    @Override
    public boolean isFinished() {
        return m_phase == Phase.DONE;
    }

    @Override
    public void end(boolean interrupted) {
        applyVolts(0);
        m_arm.stop();
        m_arm.clearAutoTuneRequest();
        if (interrupted) {
            System.out.println("[ARM TUNE] Interrupted at phase " + m_phase
                    + " — partial results: kS=" + fmt(m_resultKs)
                    + " kV=" + fmt(m_resultKv)
                    + " kG=" + fmt(m_resultKg));
            if (m_resultKs > 0 || m_resultKv > 0 || m_resultKg > 0) {
                m_arm.applyTuningResults(m_resultKs, m_resultKv, m_resultKg);
                status("Interrupted — partial results applied");
            } else {
                status("Interrupted — no results");
            }
        }
    }

    // ── Helpers ──────────────────────────────────────────────────────────────

    /** Apply voltage via duty cycle: duty = V / 12 (same approximation as before). */
    private void applyVolts(double volts) {
        m_arm.runRaw(volts / 12.0);
    }

    /** Arm velocity in RPS (absolute value, from internal motor encoder). */
    private double velRPS() {
        return m_arm.getRPM() / 60.0;
    }

    private void enterPhase(Phase p) {
        m_phase      = p;
        m_phaseStart = Timer.getFPGATimestamp();
        m_stalledConfirmed = false;
        m_stallStart = m_phaseStart;
    }

    private void startKvTest(double now) {
        m_kvPhaseStart   = now;
        m_kvVoltageAccum = 0;
        m_kvVelAccum     = 0;
        m_kvSamples      = 0;
    }

    private void status(String s) {
        SmartDashboard.putString("Arm Tune Phase", s);
        m_arm.setAutoTuneStatus(s);
    }

    private String fmt(double v) {
        return String.format("%.4f", v);
    }

    /**
     * Stall detection helper used for FIND_BOTTOM_STOP and FIND_TOP_STOP.
     * Returns true when the arm has been stalled for long enough after the grace period.
     * Also detects timeout so the test never hangs.
     */
    private boolean checkStall(double now) {
        double elapsed = now - m_phaseStart;
        if (elapsed < STOP_GRACE_S) return false;
        if (elapsed > STOP_TIMEOUT_S) {
            System.out.println("[ARM TUNE] WARNING: stop-find timeout — continuing with current position");
            return true;
        }

        if (velRPS() < STOP_STALL_VEL_RPS) {
            if (!m_stalledConfirmed) {
                m_stalledConfirmed = true;
                m_stallStart = now;
            } else if (now - m_stallStart >= STOP_STALL_CONFIRM_S) {
                return true;
            }
        } else {
            m_stalledConfirmed = false;
        }
        return false;
    }

    private void computeKv() {
        int n = KV_TEST_VOLTAGES.length;
        if (m_sumVel > 0 && n > 0) {
            double avgV   = m_sumV   / n;
            double avgVel = m_sumVel / n;
            m_resultKv = (avgV - m_resultKs) / Math.max(avgVel, 0.01);
        }
        SmartDashboard.putNumber("Arm kV", m_resultKv);
        System.out.println("[ARM TUNE] Phase 4 done — kV = " + fmt(m_resultKv) + " V·s/rot");
    }

    /**
     * Two-point kG / kS solve using hard-stop measurements.
     *
     * <p>Physics (Clockwise_Positive = arm DOWN, upward = negative motor):
     * <pre>
     *   At bottom stop, upward liftoff voltage:  V1 = kS + kG·cos(A1)
     *   At top stop, minimum upward hold voltage: V2 = kG·cos(A2) − kS
     *
     *   Sum:  V1 + V2 = kG·(cos(A1) + cos(A2))
     *   →  kG = (V1 + V2) / (cos(A1) + cos(A2))
     *   →  kS = V1 − kG·cos(A1)   [cross-check against ramp result]
     * </pre>
     */
    private void finishCharacterization() {
        double cosA1 = Math.cos(Math.toRadians(m_angleBottom));
        double cosA2 = Math.cos(Math.toRadians(m_angleTop));
        double denom = cosA1 + cosA2;

        System.out.println("[ARM TUNE] Two-point solve:");
        System.out.println("[ARM TUNE]   A1=" + fmt(m_angleBottom) + "°  cos=" + fmt(cosA1)
                + "  V1=" + fmt(m_V1));
        System.out.println("[ARM TUNE]   A2=" + fmt(m_angleTop)    + "°  cos=" + fmt(cosA2)
                + "  V2=" + fmt(m_V2));

        if (Math.abs(denom) < MIN_COS_THETA * 2) {
            System.out.println("[ARM TUNE] WARNING: cos(A1)+cos(A2) too small ("
                    + fmt(denom) + "). "
                    + "Check kArmZeroAngleDeg and angles. Using kS ramp for kS, V1 only for kG.");
            // Fallback: single-point from bottom only
            m_resultKg = (Math.abs(cosA1) > MIN_COS_THETA)
                    ? (m_V1 - m_resultKs) / cosA1
                    : 0.0;
        } else {
            m_resultKg = (m_V1 + m_V2) / denom;
            // Refine kS: take average of ramp result and two-point derivation
            double kS_twoPoint = m_V1 - m_resultKg * cosA1;
            m_resultKs = (m_resultKs + kS_twoPoint) / 2.0;
        }

        m_resultKg = Math.max(0.0, m_resultKg);
        m_resultKs = Math.max(0.0, m_resultKs);

        SmartDashboard.putNumber("Arm kS", m_resultKs);
        SmartDashboard.putNumber("Arm kG", m_resultKg);

        System.out.println("[ARM TUNE] ===== RESULTS =====");
        System.out.println("[ARM TUNE] kS = " + fmt(m_resultKs) + " V");
        System.out.println("[ARM TUNE] kV = " + fmt(m_resultKv) + " V·s/rot");
        System.out.println("[ARM TUNE] kG = " + fmt(m_resultKg) + " V");
        System.out.println("[ARM TUNE] Update Constants.java: kArmKs/kArmKv/kArmKg");

        m_arm.applyTuningResults(m_resultKs, m_resultKv, m_resultKg);
        status("DONE — kS=" + fmt(m_resultKs) + " kV=" + fmt(m_resultKv) + " kG=" + fmt(m_resultKg));
        SmartDashboard.putString("Arm Tune Phase", "DONE");

        m_phase = Phase.DONE;
    }
}
