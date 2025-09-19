package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


/**
 * Controls the motor-driven roller at the end of the robot's arm. 
 * Used to intake/extake game pieces (coral and algae).
 */
public class Roller extends SubsystemBase {

  private final TalonFX rollerMotor = new TalonFX(Constants.RollerConstants.ROLLER_MOTOR_ID, Constants.GeneralConstants.DRIVE_CANIVORE_BUS.getName());

  public State state = State.IDLE;

  public enum State {
    CORAL_EXTAKE(Constants.RollerConstants.CORAL_EXTAKE_SPEED),
    ALGAE_EXTAKE(Constants.RollerConstants.ALGAE_EXTAKE_SPEED),
    ALGAE_INTAKE(Constants.RollerConstants.ALGAE_INTAKE_SPEED),
    IDLE(Constants.RollerConstants.IDLE_SPEED);

    private double speed;

    private State(double speed) {
        this.speed = speed;
    }

    public double getSpeed() {
        return speed;
    }
  }

  public void setState(State newState) {
    state = newState;
  }

  public Command setStateCommand(State newState) {
    return Commands.runOnce(() -> setState(newState));
  }

  public Roller() {
      // Configure our arm motor
      TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();  
      talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      talonFXConfig.CurrentLimits.StatorCurrentLimit = Constants.RollerConstants.ROLLER_CURRENT_LIMIT;
  
      Slot0Configs slot0Configs = talonFXConfig.Slot0;
      slot0Configs.kP = Constants.ArmConstants.kP;
      slot0Configs.kI = Constants.ArmConstants.kI;
      slot0Configs.kD = Constants.ArmConstants.kD;
      slot0Configs.kS = Constants.ArmConstants.kS;

      rollerMotor.getConfigurator().apply(talonFXConfig);
      
      // Zeroes: arm needs to be physically straight up or code will be off
      // Needs seperate command, Needs limit switch (beam break) or absolute encoder on arm?
      rollerMotor.setPosition(0.0); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  } 
}
