package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmExample;

/**
 * Super-simple "go to a named arm state" command.
 *
 * - On initialize: tell the arm which state to go to (UP or DOWN).
 * - While scheduled: the ArmExample's periodic() does the simple seeking.
 * - isFinished: returns true when the arm reports it's at target (or timeout hits).
 * - end: stops motor output (safe to call even if already stopped).
 *
 * Notes:
 *  • This command delegates motion control to ArmExample (which currently uses a
 *    minimal open-loop/bang-bang approach). See TODOs below for next steps.
 *  • The optional timeout guards against never finishing if something jams.
 */
public class MoveArmToPositionExample extends Command {

  private final ArmExample arm;
  private final ArmExample.State targetState;

  // Optional timeout support (seconds). If <= 0, no timeout is applied.
  private final double timeoutSeconds;
  private double startTime = 0.0;

  /**
   * Create a command to move the arm to a named state with no timeout.
   */
  public MoveArmToPositionExample(ArmExample arm, ArmExample.State targetState) {
    this(arm, targetState, 0.0);
  }

  /**
   * Create a command to move the arm to a named state with an optional timeout.
   * @param timeoutSeconds if <= 0, no timeout applies
   */
  public MoveArmToPositionExample(ArmExample arm, ArmExample.State targetState, double timeoutSeconds) {
    this.arm = arm;
    this.targetState = targetState;
    this.timeoutSeconds = timeoutSeconds;
    addRequirements(arm); // Scheduler will manage exclusivity
  }

  @Override
  public void initialize() {
    // Record start time for optional timeout.
    startTime = Timer.getFPGATimestamp();

    // Kick off the motion by setting the goal in the subsystem.
    arm.setState(targetState);

    // TODO (Logging): Log "moving to UP/DOWN", current angle, target angle.
    // TODO (Safety): Check any interlocks here (e.g., don't move if elevator is low).
  }

  @Override
  public void execute() {
    // Nothing required here since ArmExample.periodic() is doing the actual seek.
    // Leaving this empty keeps the command easy to reason about.

    // TODO (Advanced): If you move control from subsystem periodic() into a closed-loop
    // command-based controller (PID/FF on-Rio), this is where you'd calculate outputs
    // and call `arm.setVoltage(...)` each loop.
  }

  @Override
  public boolean isFinished() {
    // Finish if we reached the target...
    if (arm.isAtTarget()) {
      return true;
    }
    // ...or if an optional timeout is set and has elapsed.
    if (timeoutSeconds > 0.0) {
      double elapsed = Timer.getFPGATimestamp() - startTime;
      if (elapsed >= timeoutSeconds) {
        return true;
      }
    }
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    // Always stop applying output when the command ends or is interrupted.
    arm.stop();

    // TODO (Logging): Log end state, whether interrupted, final angle, error, elapsed time.
    // TODO (Fallback): If timed out, consider setting a safe "hold" voltage or gravity FF.
  }

  // ---------------- Convenience factories ----------------

  /** Convenience: Go to UP with no timeout. */
  public static MoveArmToPositionExample up(ArmExample arm) {
    return new MoveArmToPositionExample(arm, ArmExample.State.UP);
  }

  /** Convenience: Go to DOWN with no timeout. */
  public static MoveArmToPositionExample down(ArmExample arm) {
    return new MoveArmToPositionExample(arm, ArmExample.State.DOWN);
  }

  /** Convenience: Go to UP with a timeout. */
  public static MoveArmToPositionExample up(ArmExample arm, double timeoutSeconds) {
    return new MoveArmToPositionExample(arm, ArmExample.State.UP, timeoutSeconds);
  }

  /** Convenience: Go to DOWN with a timeout. */
  public static MoveArmToPositionExample down(ArmExample arm, double timeoutSeconds) {
    return new MoveArmToPositionExample(arm, ArmExample.State.DOWN, timeoutSeconds);
  }
}
