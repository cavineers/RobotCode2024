package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmBase extends SubsystemBase {

    
    public enum BaseMotorState {
        ON,
        OFF,
        REVERSED
    }
    
    // Motor Initialization
    public CANSparkMax baseMotor = new CANSparkMax(Constants.ArmBase.BaseMotor, MotorType.kBrushless);
    
    // Starts motors in their off state
    public BaseMotorState baseMotorState = BaseMotorState.OFF;
    
    // Motor sparkmax settings
    public ArmBase() {
        this.baseMotor.setIdleMode(IdleMode.kBrake);
        
        this.baseMotor.setSmartCurrentLimit(51);
    }
    
    public void setBaseMotorState(BaseMotorState state) {
        // set the current state
        this.baseMotorState = state;
        
        // set motor state
        switch (state) {
            case ON:
            // On
            this.baseMotor.set(Constants.ArmBase.BaseMotorSpeedForwards);
            break;
            
            case OFF:
            // Off
            this.baseMotor.set(0);
            break;
            
            case REVERSED:
            // Reversed
                this.baseMotor.set(Constants.ArmBase.BaseMotorSpeedBackwards);
                break;

                default:
                this.setBaseMotorState(BaseMotorState.OFF);
            }
        }
        
    public double getBaseMotorPosition() {
        return this.baseMotor.getEncoder().getPosition();
    }

    public double getBaseMotorSpeed() {
        return this.baseMotor.get();
    }
    
    public BaseMotorState getBaseMotorState() {
        return this.baseMotorState;
    }
    
    public void setBaseMotorPosition(double position) {
        this.baseMotor.getEncoder().setPosition(position);
    }
    
    public void periodic() {}
    
}