package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private final TalonFX armMotor = new TalonFX(Constants.ArmConstants.ARM_MOTOR_ID, Constants.GeneralConstants.DRIVE_CANIVORE_BUS.getName());
    
    public State state = State.UP;
    
    public enum State {
        LEFT(Constants.ArmConstants.POSITION_LEFT),
        RIGHT(Constants.ArmConstants.POSITION_RIGHT),
        UP(Constants.ArmConstants.POSITION_UP);

        private Rotation2d rotation;

        private State(Rotation2d rotation) {
            this.rotation = rotation;
        }

        public Rotation2d getRotation2d() {
            return rotation;
        }
    }

    
    public Arm() {
        TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();  
        talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfig.CurrentLimits.StatorCurrentLimit = Constants.ArmConstants.ARM_CURRENT_LIMIT; // Set the stator current limit to 30
    
        Slot0Configs slot0Configs = talonFXConfig.Slot0;
        slot0Configs.kP = Constants.ArmConstants.kP;
        slot0Configs.kI = Constants.ArmConstants.kI;
        slot0Configs.kD = Constants.ArmConstants.kD;
        slot0Configs.kS = Constants.ArmConstants.kS;
        // Add more?
        armMotor.getConfigurator().apply(talonFXConfig);
        
        armMotor.setPosition(0.0);  // Zeroes: arm needs to be physically straight up or code will be off
        // Needs seperate command, Needs limit switch (beam break) or absolute encoder on arm
    }

    public void setState(State newState) {
        state = newState;
        setArmPosition(state.getRotation2d().getRotations());
    }

    public Command setStateCommand(State newState) {
        return Commands.runOnce(() -> setState(newState));
    }

    private void setArmPosition(double position) {
        armMotor.setControl(new PositionDutyCycle(position));
    }

    public void setVoltageOutput(double output) {
        armMotor.setControl(new DutyCycleOut(output));
    }
}
    
