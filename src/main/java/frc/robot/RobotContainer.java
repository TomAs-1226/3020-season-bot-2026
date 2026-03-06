// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmCharacterizeCommand;
import frc.robot.commands.AutoDeployIntakeCommand;
import frc.robot.commands.DeployDownCommand;
import frc.robot.commands.DeployUpCommand;
import frc.robot.commands.HomeArmCommand;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.RunIntakeReverseCommand;
import frc.robot.commands.RunMotorGroup1Command;
import frc.robot.commands.RunMotorGroup2ReverseCommand;
import frc.robot.commands.ShootAutoFeedCommand;
import frc.robot.commands.ShootStowArmCommand;
import frc.robot.commands.ShootV1Command;
import frc.robot.commands.UnjamIntakeCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeDeploySubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MotorGroup1Subsystem;
import frc.robot.subsystems.MotorGroup2Subsystem;
import frc.robot.subsystems.PowerManagementSubsystem;
import frc.robot.subsystems.TelemetrySubsystem;

/**
 * ============================================================================
 *                    TEAM 3020 - ROBOT CONTAINER
 * ============================================================================
 *
 * CONTROLLER: PS5 DualSense on port 0
 *
 * DRIVING:
 *   Left Stick   = Field-centric translation (forward / strafe)
 *   Right Stick  = Rotation
 *   L3 (push)    = X-lock wheels (brake)
 *   Options      = Re-zero field heading (press if gyro drifts)
 *
 * ── SHOOTING ──────────────────────────────────────────────────────────────
 *   L1           = SHOOT (hold) — flywheel spins up; auto-feeds when at RPM.
 *                  Arm stows while shooting (pushes balls toward indexer).
 *                  "Shoot Status" on SmartDashboard shows live spinup progress.
 *   Cross (X)    = Feeder manual override (hold) — bypasses RPM check. Use to
 *                  clear a jam or force-feed without waiting for spinup.
 *   R1           = Shooter reverse (hold) — clears flywheel jams.
 *
 * ── INTAKE ────────────────────────────────────────────────────────────────
 *   Square       = Auto intake TOGGLE — press once to deploy arm + run rollers;
 *                  press again to stop. FIRST press of each power cycle also
 *                  homes the arm to the bumpers for accurate encoder zeroing.
 *   Triangle     = Intake rollers only (hold) — rollers without moving the arm.
 *                  Use when arm is already deployed and you want manual roller control.
 *   Circle       = Quick eject (hold) — intake roller reverse only.
 *                  Spits out a game piece without moving the arm or feeder.
 *
 * ── OUTTAKE / CLEAR ───────────────────────────────────────────────────────
 *   L2           = FULL OUTTAKE (hold) — intake roller + feeder both reverse.
 *                  Clears the entire path back through the intake.
 *   R3 (push)    = Unjam intake — nudges arm + reverses roller; auto-stops.
 *
 * ── ARM ───────────────────────────────────────────────────────────────────
 *   D-pad DOWN   = Deploy arm. First press homes to bumper then deploys.
 *                  Subsequent presses deploy directly (already homed).
 *   D-pad UP     = Stow arm.
 *   Create       = Home arm (manual) — drives arm to bumper, zeros encoder.
 *                  Press any time arm position looks wrong mid-match.
 *
 * ── DRIVE ─────────────────────────────────────────────────────────────────
 *   Left Stick   = Field-centric translation (push forward = drive forward)
 *   Right Stick  = Rotation
 *   L3 (push)    = X-lock wheels — robot resists being pushed
 *   Options      = Re-zero heading — current facing becomes field "forward"
 *
 * ── TUNING (pit / pre-match only) ─────────────────────────────────────────
 *   Touchpad     = Arm auto-tune (hold — arm WILL move through full range!)
 *
 * CAN IDs (all RIO bus ""):
 *   Feeder  : 40 (hopper), 41 (feed roller)
 *   Shooter : 30 (leader), 31 (follower)
 *   Intake  : 45 (Falcon 500)
 *   Arm     : 46 (Kraken X60)
 *   Swerve FL: steer=11, drive=12, encoder=13
 *   Swerve FR: steer=20, drive=21, encoder=22
 *   Swerve BL: steer=14, drive=15, encoder=16
 *   Swerve BR: steer=17, drive=18, encoder=19
 *   Pigeon2  : 23
 *
 * @author Baichen Yu
 * ============================================================================
 */
public class RobotContainer {

  // ===== DRIVE SPEED LIMITS =====
  private final double MaxSpeed      = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // ~4.83 m/s
  // Increased from 0.75 → 1.25 rot/s for more responsive rotation feel.
  // Lower the deadband % and raise this if rotation still feels sluggish.
  private final double MaxAngularRate = RotationsPerSecond.of(1.25).in(RadiansPerSecond);

  // ===== SWERVE REQUESTS =====
  // Field-centric drive: translation is relative to the field, not the robot heading.
  // Deadband set to 5% (was 10%) so small stick inputs are not ignored.
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.05)
      .withRotationalDeadband(MaxAngularRate * 0.05)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  // X-lock: all wheels point inward to resist being pushed
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  // ===== DRIVETRAIN =====
  private final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();

  // ===== MECHANISM SUBSYSTEMS =====
  private final MotorGroup1Subsystem    m_feeder    = new MotorGroup1Subsystem();
  private final MotorGroup2Subsystem    m_shooter   = new MotorGroup2Subsystem();
  private final IntakeSubsystem         m_intake    = new IntakeSubsystem();
  private final IntakeDeploySubsystem   m_intakeArm = new IntakeDeploySubsystem();
  private final PowerManagementSubsystem m_power    = new PowerManagementSubsystem();
  private final TelemetrySubsystem      m_telemetry = new TelemetrySubsystem();

  // ===== CONTROLLER =====
  private final CommandPS5Controller m_controller =
      new CommandPS5Controller(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    System.out.println("============================================");
    System.out.println("        TEAM 3020 ROBOT STARTING UP");
    System.out.println("============================================");

    m_intake.setPowerManagement(m_power);

    configureBindings();

    System.out.println("[TEAM 3020] Ready! Max speed: " + MaxSpeed + " m/s");
  }

  private void configureBindings() {

    // ===== DRIVETRAIN (sticks) =====
    // Left stick Y  = forward/back  (negated: push up = drive forward)
    // Left stick X  = strafe        (negated: push left = strafe left)
    // Right stick X = rotate        (negated: push right = clockwise)
    // NOTE: if the robot drives in the wrong direction on one axis, flip that axis' sign here.
    // NOTE: if wheels form an X shape during rotation, re-run Tuner X wheel alignment.
    m_drivetrain.setDefaultCommand(
        m_drivetrain.applyRequest(() ->
            drive.withVelocityX   (-m_controller.getLeftY()  * MaxSpeed)
                 .withVelocityY   (-m_controller.getLeftX()  * MaxSpeed)
                 .withRotationalRate(-m_controller.getRightX() * MaxAngularRate)
        )
    );

    // L3 (push left stick) = X-lock brake
    m_controller.L3().whileTrue(m_drivetrain.applyRequest(() -> brake));

    // Options = re-zero field heading (current facing becomes "forward")
    m_controller.options().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldCentric()));
    // ===== SHOOT (L1) — THREE INDEPENDENT BINDINGS =====
    // Each binding is INDEPENDENT: interrupting one does NOT affect the others.
    // This prevents manual feeder (Cross), outtake (L2), etc. from killing the shooter.

    // 1) SHOOTER: spins up and holds target RPM. NOTHING can interrupt this except
    //    releasing L1 or pressing R1 (shooter reverse, which requires shooter).
    m_controller.L1()
        .whileTrue(new ShootV1Command(m_shooter));

    // 2) AUTO-FEED: waits for shooter on-target (or 2.5s safety timeout at ≥60% RPM),
    //    then runs feeder until L1 is released. If Cross is pressed, this feeder command
    //    is interrupted but the shooter keeps spinning independently.
    m_controller.L1()
        .whileTrue(new ShootAutoFeedCommand(m_shooter, m_feeder));

    // 3) ARM STOW: waits INTERNALLY for shooter on-target (no premature timeout),
    //    then slowly stows arm to push balls toward the indexer.
    //    Completely independent of shooter and feeder.
    m_controller.L1()
        .whileTrue(new ShootStowArmCommand(m_intakeArm, m_shooter));

    // ===== FEEDER MANUAL OVERRIDE (Cross) =====
    // Run feeder alone without spinup check — useful for clearing jams.
    m_controller.cross()
        .whileTrue(new RunMotorGroup1Command(m_feeder));

    // ===== SHOOTER REVERSE (R1) =====
    m_controller.R1()
        .whileTrue(new RunMotorGroup2ReverseCommand(m_shooter));

    // ===== INTAKE — TOGGLE (Square) =====
    // First press: homes arm to bumper (encoder zero), then deploys + runs rollers.
    // Subsequent presses: deploy + run rollers directly (already homed this power cycle).
    m_controller.square()
        .toggleOnTrue(new ConditionalCommand(
            new AutoDeployIntakeCommand(m_intake, m_intakeArm),
            new HomeArmCommand(m_intakeArm).andThen(new AutoDeployIntakeCommand(m_intake, m_intakeArm)),
            m_intakeArm::hasHomed));

    // ===== INTAKE EJECT / OUTTAKE (Circle) =====
    // Run intake roller backwards to spit out a game piece (hold).
    m_controller.circle()
        .whileTrue(new RunIntakeReverseCommand(m_intake));

    // ===== INTAKE ROLLERS ONLY (Triangle) =====
    // Rollers without moving the arm (hold).
    m_controller.triangle()
        .whileTrue(new RunIntakeCommand(m_intake));

    // ===== FULL OUTTAKE — intake reverse + feeder reverse (L2) =====
    // Clears the entire path: intake roller + feeder both run backwards while holding L2.
    m_controller.L2()
        .whileTrue(new RunIntakeReverseCommand(m_intake)
            .alongWith(m_feeder.run(m_feeder::runMotorsReverse)));

    // ===== UNJAM INTAKE (R3 — press right stick) =====
    // Nudges the arm up/down while reversing the roller.  Auto-stops when done.
    m_controller.R3()
        .onTrue(new UnjamIntakeCommand(m_intakeArm, m_intake));

    // ===== ARM DEFAULT: hold at last commanded position via closed-loop =====
    // Keeps arm locked wherever it was last commanded even when no button is pressed.
    m_intakeArm.setDefaultCommand(m_intakeArm.run(m_intakeArm::holdPosition));

    // ===== ARM CONTROL (D-pad) =====
    m_controller.povUp()
        .onTrue(new DeployUpCommand(m_intakeArm));
    // D-pad Down: homes to bumper on first press, deploys directly thereafter.
    m_controller.povDown()
        .onTrue(new ConditionalCommand(
            new DeployDownCommand(m_intakeArm),
            new HomeArmCommand(m_intakeArm).andThen(new DeployDownCommand(m_intakeArm)),
            m_intakeArm::hasHomed));

    // ===== HOME ARM (Create) =====
    // Drives arm slowly to bumper, zeros encoder, marks homed.
    // Press any time arm position looks wrong (e.g. arm moved by hand before power-on).
    m_controller.create()
        .onTrue(new HomeArmCommand(m_intakeArm));

    // ===== RE-SEED FIELD CENTRIC (Touchpad) =====
    // Second re-zero binding — same as Options but reachable mid-match with one hand.
    m_controller.touchpad()
        .onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldCentric()));

    // ===== ARM CHARACTERIZATION — TEMP (D-pad Left) =====
    // TODO: REMOVE BEFORE MATCH — pit/practice only trigger for arm auto-tune.
    // ARM WILL MOVE through its full range — ensure clearance before running!
    m_controller.povLeft()
        .whileTrue(new ArmCharacterizeCommand(m_intakeArm));

    // ===== ARM CHARACTERIZATION (Shuffleboard button) =====
    // Toggle "Request Auto Tune" boolean in Shuffleboard "Arm" tab.
    // Set it to true to start; command resets it to false when done.
    new Trigger(m_intakeArm::isAutoTuneRequested)
        .onTrue(new ArmCharacterizeCommand(m_intakeArm));

    // Defense mode trigger removed — feeder current was spiking voltage sag to 10V
    // and killing the shooter mid-match.  Power restrictions removed per team decision.

    System.out.println("[TEAM 3020] All bindings configured.");
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
