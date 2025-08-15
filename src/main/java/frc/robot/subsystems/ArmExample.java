package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Simplest possible "two-position" arm using a Kraken (Talon FX).
 * - Open-loop: drives the motor a fixed percent until within a small angle tolerance.
 * - Two named states via enum: UP (90°) and DOWN (35°).
 * 
 * How'd you'd use it, In RobotContainer:
 *
 * private final ArmSubsystem arm = new ArmSubsystem();
 *
 * upButton.onTrue(Commands.runOnce(arm::goUp, arm));
 * downButton.onTrue(Commands.runOnce(arm::goDown, arm));
 * 
 * Notes on “future sophistication”
 *
 * PID + Feedforward: Replace the simple bang-bang in periodic() with either 
 * WPILib’s ProfiledPIDController + ArmFeedforward (nice and portable) or 
 * CTRE’s MotionMagic with a static+gravity FF term. 
 * That will hold angle precisely and move smoothly without overshoot.
 *
 * Absolute position at boot: Add a CANcoder on the joint (or a DutyCycleEncoder on DIO). 
 * On robotInit, read absolute angle, compute armZeroOffsetDeg, and 
 * you’re done—no manual zeroing required.
 *
 * Soft limits & safety: Once your sensor is trustworthy, enable software limits and 
 * consider reducing kSeekPercentOutput to the minimum that still moves reliably.
 */
public class ArmExample extends SubsystemBase {

  /** Arm states we support today. */
  public enum State {
    UP,   // 90 degrees (vertical)
    DOWN  // 35 degrees (lower position)
  }

  // -------------------- USER CONSTANTS (adjust these) --------------------

  // CAN ID of the Kraken/TalonFX
  private static final int kArmMotorId = 10;

  // Gear ratio: motor rotations per ONE arm revolution (e.g., 100:1 -> 100.0)
  // This must include all stages from motor shaft to arm pivot.
  private static final double kMotorRotationsPerArmRotation = 100.0;

  // Mechanical range and targets (degrees). UP is vertical (90°), DOWN is 35°
  private static final double kTargetUpDeg   = 90.0;
  private static final double kTargetDownDeg = 35.0;

  // Safety bounds (soft range we "expect"); used to clamp targets/open-loop motion
  private static final double kMinArmDeg = 0.0;    // don't go below flat if possible
  private static final double kMaxArmDeg = 120.0;  // a bit above UP to be safe

  // How hard to drive when moving (keep small!)
  private static final double kSeekPercentOutput = 0.15; // 15% duty
  // Angle tolerance to consider "at target"
  private static final double kAngleToleranceDeg = 1.5;

  // -------------------- HARDWARE --------------------
  private final TalonFX armMotor = new TalonFX(kArmMotorId);
  private final DutyCycleOut dutyOut = new DutyCycleOut(0.0);

  // -------------------- STATE --------------------
  private State goalState = State.DOWN;   // default starting goal
  private double targetDeg = kTargetDownDeg;

  // We treat the TalonFX integrated position as "zero at boot" and add an offset you can set.
  // Call zeroToCurrentPosition() once you place the arm in a known pose (e.g., at DOWN).
  private double armZeroOffsetDeg = 0.0;

  public ArmExample() {
    // Minimal, safe-ish motor setup
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = 35.0;   // simple current limit
    //cfg.CurrentLimits.SupplyCurrentThreshold = 60.0;
    //cfg.CurrentLimits.SupplyTimeThreshold = 0.2;
    // TODO: cfg.Voltage.PeakForwardVoltage/ReverseVoltage if you want extra safety
    // TODO: cfg.SoftwareLimitSwitch to add real soft limits tied to integrated sensor

    armMotor.getConfigurator().apply(cfg);

    // Optional: invert if your wiring makes UP go the wrong way
    // armMotor.setInverted(true);
  }

  // -------------------- Public API --------------------

  /** Command the arm to the UP position (90°). */
  public void goUp() {
    setState(State.UP);
  }

  /** Command the arm to the DOWN position (35°). */
  public void goDown() {
    setState(State.DOWN);
  }

  /** Toggle between UP and DOWN. */
  public void toggle() {
    setState(goalState == State.UP ? State.DOWN : State.UP);
  }

  /** Set the goal state explicitly. */
  public void setState(State state) {
    goalState = state;
    targetDeg = (state == State.UP) ? kTargetUpDeg : kTargetDownDeg;
    // Clamp to safety range
    targetDeg = clamp(targetDeg, kMinArmDeg, kMaxArmDeg);
  }

  /** Returns true if we're within tolerance of the current target. */
  public boolean isAtTarget() {
    return Math.abs(getArmAngleDeg() - targetDeg) <= kAngleToleranceDeg;
  }

  /** Stop applying output to the motor immediately. */
  public void stop() {
    armMotor.setControl(dutyOut.withOutput(0.0));
  }

  /**
   * Treat the arm's current pose as zero.
   * Use this after you place the arm at a known angle (e.g., DOWN = 35°),
   * then call setZeroAs(knownAngleDeg) so internal math aligns.
   */
  public void zeroToCurrentPosition() {
    armZeroOffsetDeg = -rawArmAngleDeg();
  }

  /** Adjust zero so that the current pose reads as the provided angle (e.g., 35.0). */
  public void setZeroAs(double knownAngleDeg) {
    armZeroOffsetDeg = knownAngleDeg - rawArmAngleDeg();
  }

  // -------------------- Periodic control loop (very simple) --------------------

  @Override
  public void periodic() {
    // Ultra-simple "bang-bang" style seek:
    //  - If we're outside the tolerance, drive a small constant in the correct direction.
    //  - Else, stop.
    double currentDeg = getArmAngleDeg();
    double error = targetDeg - currentDeg;

    // Simple soft range guard: if we're outside, bias motion back in-bounds
    if (currentDeg < kMinArmDeg - 2.0) {
      armMotor.setControl(dutyOut.withOutput(+Math.abs(kSeekPercentOutput)));
      return;
    } else if (currentDeg > kMaxArmDeg + 2.0) {
      armMotor.setControl(dutyOut.withOutput(-Math.abs(kSeekPercentOutput)));
      return;
    }

    if (Math.abs(error) <= kAngleToleranceDeg) {
      // Close enough
      armMotor.setControl(dutyOut.withOutput(0.0));
    } else {
      double direction = Math.signum(error);
      armMotor.setControl(dutyOut.withOutput(direction * kSeekPercentOutput));
    }

    // TODO (Logging): Publish currentDeg, targetDeg, error to SmartDashboard/NT for tuning
  }

  // -------------------- Sensing & math --------------------

  /**
   * Returns the arm angle in degrees, including your configured zero offset.
   * Uses TalonFX integrated rotor position and the gear ratio.
   *
   * NOTE: This is RELATIVE to when the robot booted unless you call zeroToCurrentPosition()
   * or setZeroAs(). For true power-on absolute, add a CANcoder or duty-cycle absolute encoder.
   */
  public double getArmAngleDeg() {
    return rawArmAngleDeg() + armZeroOffsetDeg;
  }

  /** Raw arm angle from integrated sensor with no offset (degrees). */
  private double rawArmAngleDeg() {
    // Motor rotations since boot:
    double motorRot = armMotor.getPosition().getValueAsDouble(); // rotations
    // Convert to arm rotations using your gear ratio, then to degrees:
    double armRot = motorRot / kMotorRotationsPerArmRotation;
    return armRot * 360.0;
  }

  /** Utility clamp. */
  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }

  // -------------------- Where to go next (quick notes) --------------------
  // TODO (Absolute Homing): Add a limit switch or absolute encoder (CANcoder / DutyCycleEncoder)
  //       so the arm knows its angle at power-up without manual zeroing.
  //
  // TODO (Closed-loop control): Replace bang-bang with:
  //       - WPILib PID: ProfiledPIDController for motion constraints
  //       - CTRE Motion Magic or PositionClosedLoop for TalonFX-native control
  //
  // TODO (Feedforward): Use WPILib ArmFeedforward(ks, kg, kv, ka) to add gravity compensation,
  //       especially important for holding positions and smooth motion.
  //
  // TODO (Soft limits): Enable motor-controller soft limits tied to your sensor for safety.
  //
  // TODO (Safety): Add current limiting, voltage compensation, and a deadman/disable path
  //       from commands so the driver can stop the arm instantly.
  //
  // TODO (Sim): Implement arm physics in simulation using WPILib’s arm sim classes for testing.
}
