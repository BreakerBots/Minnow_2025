package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.BreakerLib.sensors.BreakerDigitalSensor;


/**
 * Controls the motor-driven roller at the end of the robot's arm. 
 * Used to intake/extake game pieces (coral and algae).
 */
public class Roller extends SubsystemBase {

  private final TalonFX rollerMotor = new TalonFX(Constants.RollerConstants.ROLLER_MOTOR_ID, Constants.GeneralConstants.SUPERSTRUCTURE_CANIVORE_BUS.getName());
  // private final BreakerDigitalSensor beamBreak;
  
  
  // also gonna need a CANdi (probably)

  public State state = State.IDLE;


  public enum State {
    CORAL_EXTAKE(Constants.RollerConstants.CORAL_EXTAKE_SPEED),
    ALGAE_EXTAKE(Constants.RollerConstants.ALGAE_EXTAKE_SPEED),
    ALGAE_INTAKE(Constants.RollerConstants.ALGAE_INTAKE_SPEED),
    ALGAE_STOW(Constants.RollerConstants.ALGAE_STOW_SPEED),
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
    setSpeed(state.getSpeed());
  }

  public Command setStateCommand(State newState) {
    return Commands.runOnce(() -> setState(newState));
  }

  public void setSpeed(double speed) {
    rollerMotor.setControl(new DutyCycleOut(speed));
  }

  public Command setSpeedCommand(double speed) {
    return Commands.runOnce(() -> setSpeed(speed));
  }

  public Roller() {
    
    // Configure our roller+ motor
      TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();  
      talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      talonFXConfig.CurrentLimits.StatorCurrentLimit = Constants.RollerConstants.ROLLER_CURRENT_LIMIT;

      rollerMotor.getConfigurator().apply(talonFXConfig);
  }

  public boolean hasAlgae = false;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if (beamBreak.getAsBoolean()) {
    //   hasAlgae = true;
    //   setStateCommand(Roller.State.IDLE);
    // }
  } 
}
