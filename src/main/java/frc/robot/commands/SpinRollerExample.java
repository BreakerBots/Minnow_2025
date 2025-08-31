// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.RollerExample;

// /**
//  * Parameterized "spin roller" command.
//  * - Sets the roller to the requested state on initialize.
//  * - Runs until interrupted (e.g., whileTrue()) or until the optional endOnIdle flag is used.
//  * - Stops the roller on end().
//  *
//  * TODO ideas:
//  *  - Add optional per-state speed overrides via constructor args.
//  *  - Add interlocks (e.g., require Arm at safe angle) before enabling motion.
//  *  - Add logging/telemetry hooks.
//  */
// public class SpinRollerExample extends Command {
//   private final RollerExample roller;
//   private final RollerExample.State state;

//   public SpinRollerExample(RollerExample roller, RollerExample.State state) {
//     this.roller = roller;
//     this.state = state;
//     addRequirements(roller);
//   }

//   @Override
//   public void initialize() {
//     roller.setState(state);
//     // TODO: Log start (state).
//   }

//   @Override
//   public boolean isFinished() {
//     // Keep running until interrupted (typical whileTrue binding).
//     return false;
//   }

//   @Override
//   public void end(boolean interrupted) {
//     roller.setState(RollerExample.State.IDLE);
//   }

//   // --------- Named factories for readability in bindings ---------
//   public static SpinRollerExample algaeIntake(RollerExample roller) {
//     return new SpinRollerExample(roller, RollerExample.State.ALGAE_INTAKE);
//   }
//   public static SpinRollerExample coralExtake(RollerExample roller) {
//     return new SpinRollerExample(roller, RollerExample.State.CORAL_EXTAKE);
//   }
//   public static SpinRollerExample algaeExtake(RollerExample roller) {
//     return new SpinRollerExample(roller, RollerExample.State.ALGAE_EXTAKE);
//   }
//   public static SpinRollerExample stop(RollerExample roller) {
//     return new SpinRollerExample(roller, RollerExample.State.IDLE);
//   }
// }
