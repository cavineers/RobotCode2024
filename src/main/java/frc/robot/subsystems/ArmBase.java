package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmBase extends SubsystemBase {

    public void periodic() {}

     public enum MotorState {
        ON,
        OFF,
        REVERSED
    }

    // Motor Initialization
    public CANSparkMax baseMotor = new CANSparkMax(Constants.ArmBase.BaseMotor, MotorType.kBrushless);
    
    // Starts motors in their off state
    public MotorState baseMotorState = MotorState.OFF;

    // Motor sparkmax settings
    public ArmBase() {
        this.baseMotor.setIdleMode(IdleMode.kBrake);

        this.baseMotor.setSmartCurrentLimit(51);
       
    }
    
    public void setBaseMotorState(MotorState state) {
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
                this.setBaseMotorState(MotorState.OFF);
        }
    }

    public double getBaseMotorPosition() {
        return this.baseMotor.getEncoder().getPosition();
    }

    public void setBaseMotorPosition(double position) {
        this.baseMotor.getEncoder().setPosition(position);
    }
}