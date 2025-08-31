package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private final TalonFX armMotor = new TalonFX(31,"drive_canivore");
    
    public Arm() {
        TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();  
        talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfig.CurrentLimits.StatorCurrentLimit = 30.0; // Set the stator current limit to 30
    
        Slot0Configs slot0Configs = talonFXConfig.Slot0;
        slot0Configs.kP = 0.2;
        slot0Configs.kI = 0.003;
        slot0Configs.kD = 0.008;
        slot0Configs.kS = 0.05;
    
        armMotor.getConfigurator().apply(talonFXConfig);
        
        armMotor.setPosition(0.0);
    }
    
    public void setArmPosition(double position) {
        armMotor.setControl(new PositionDutyCycle(position));
    }
    
  
    
}
    
