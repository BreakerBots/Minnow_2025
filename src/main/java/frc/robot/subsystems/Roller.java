package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Controllers the motor-driven roller at the end of the robot's arm. 
 * Used to intake/extake game pieces (coral and algae).
 */
public class Roller extends SubsystemBase {

  public enum State {
    CORAL_EXTAKE,
    ALGAE_EXTAKE,
    ALGAE_INTAKE,
    IDLE,
  }

  private State state = State.IDLE;

  public void setState(State newState) {
    state = newState;
  }

  public Command setStateCommand(State newState) {
    return Commands.runOnce(() -> setState(newState));
  }

  public Roller() {
      
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  } 
}
