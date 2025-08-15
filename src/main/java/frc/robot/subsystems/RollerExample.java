package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Simplest possible roller subsystem:
 * - One Kraken (TalonFX) running open-loop percent output.
 * - Four named states:
 *   • CORAL_EXTAKE  -> spin forward (slow)
 *   • ALGAE_EXTAKE  -> spin reverse (slow)
 *   • ALGAE_INTAKE  -> spin forward (slow)
 *   • IDLE          -> stopped
 *
 * Notes:
 *  • For now, all spinning states use the same "slowish" speed. See TODOs for
 *    splitting per-state speeds and adding closed-loop velocity control later.
 *  • Direction assumptions may need flipping depending on your wiring—use
 *    kInverted or swap signs below.
 * 
 * TODO (Per-state speeds): Split kSpinPercent into kCoralExtakePct, kAlgaeIntakePct, kAlgaeExtakePct.
 * TODO (Closed-loop velocity): Use CTRE VelocityClosedLoop or WPILib PID to hit exact roller RPMs.
 * TODO (Sensors): Add a beam-break or current-spike detector to auto-stop when game piece is acquired/ejected.
 * TODO (Smart current/voltage): Enable voltage compensation and ramp rates for smoother starts.
 * TODO (Commands): Add simple commands like StartAlgaeIntake, StartCoralExtake, StartAlgaeExtake, StopRoller.
 * TODO (Sim): Provide a trivial simulation model if you want to test logic in WPILib sim.
 */
public class RollerExample extends SubsystemBase {

  public enum State {
    CORAL_EXTAKE,
    ALGAE_EXTAKE,
    ALGAE_INTAKE,
    IDLE
  }

  // -------------------- User-tunable constants (adjust on robot) --------------------

  /** CAN ID of the roller Kraken/TalonFX. */
  private static final int kRollerMotorId = 21;

  /** Invert if your "forward" direction is physically opposite. */
  private static final boolean kInverted = false;

  /** One shared "slowish" speed (0..1). Split per-state later if needed. */
  private static final double kSpinPercent = 0.20;

  /** Optional: current limits for basic safety. */
  private static final double kSupplyLimit = 30.0;

  // -------------------- Hardware --------------------
  private final TalonFX rollerMotor = new TalonFX(kRollerMotorId);
  private final DutyCycleOut dutyOut = new DutyCycleOut(0.0);


  private State state = State.IDLE;

  /** Set the desired roller state. */
  public void setState(State newState) {
    state = newState;
  }

  /** Get the current roller state. */
  public State getState() {
    return state;
  }


  public RollerExample() {
    // Minimal motor configuration for a roller
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast; // rollers usually okay to coast
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = kSupplyLimit;
    //cfg.CurrentLimits.SupplyCurrentThreshold = kSupplyThreshold;
    //cfg.CurrentLimits.SupplyTimeThreshold = kSupplyTime;

    rollerMotor.getConfigurator().apply(cfg);
    rollerMotor.setInverted(kInverted);

    // TODO: cfg.Voltage.PeakForwardVoltage/PeakReverseVoltage to cap output for safety
    // TODO: Use stator current limits if you want tighter motor-protection behavior
  }


  @Override
  public void periodic() {
    double output;

    // Map state -> percent output.
    // Convention here:
    //  - "Forward" (positive) is used for CORAL_EXTAKE and ALGAE_INTAKE
    //  - "Reverse" (negative) is used for ALGAE_EXTAKE
    // Flip kInverted above if your wiring makes this backwards in reality.
    switch (state) {
    case CORAL_EXTAKE:
        output = +kSpinPercent;
        break;
    case ALGAE_INTAKE:
        output = +kSpinPercent;
        break;
    case ALGAE_EXTAKE:
        output = -kSpinPercent;
        break;
    case IDLE:
    default:
        output = 0.0;
        break;
    }

    rollerMotor.setControl(dutyOut.withOutput(output));

    // TODO (Telemetry): Publish state and output to SmartDashboard/NT for driver feedback.
    // TODO (Coordination): If this roller sits on the arm, consider interlocks with arm positions.
  }
}
