package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private final TalonFX armMotor = new TalonFX(Constants.ArmConstants.ARM_MOTOR_ID,Constants.GeneralConstants.DRIVE_CANIVORE_BUS);
    
    public Arm() {
        TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();  
        talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfig.CurrentLimits.StatorCurrentLimit = Constants.ArmConstants.ARM_CURRENT_LIMIT; 
    
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
    
    public void setArmPosition(double position) {
        armMotor.setControl(new PositionDutyCycle(position));
    }
    
  
    
}
    
