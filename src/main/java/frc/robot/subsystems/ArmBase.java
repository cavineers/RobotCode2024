// Gantry Movement
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmBase extends SubsystemBase {

// Motor State for Arm Base
    public enum ArmBaseMotorState() {
        ON,
        OFF,
        REVERSED
    }

// Starts Motor
    public CANSparkMax m_armBaseMotor = new CANSparkMax(Constants.Arm.ArmBaseMotor, MotorType.kBrushless);

    public ArmBaseMotorState m_armBaseMotorState = ArmBaseMotorState.OFF;

    public ArmBase(){
        this.ArmBaseMotor.setIdleMode(IdleMode.kBrake);
        this.ArmBaseMotor.setSmartCurrentLimit(41); ///TBD
    }

    public void setArmBaseMotorState(ArmBaseMotorState state){
        this.ArmBaseMotor = state;
        switch(state){
            case ON:
                this.ArmBaseMotor.set(Constants.Arm.ArmBaseForwardSpeed);
                break;
            case OFF: 
                this.ArmBaseMotor.set(speed: 0.0);
            case REVERSED: 
                this.ArmBaseMotor.set(Constants.Arm.ArmBaseBackwardsSpeed);
                break;
            default: 
                this.setArmBaseMotorState(ArmBaseMotorState.OFF);
        }
    }

    public ArmBaseMotorState getArmBaseMotorState(){
        return this.ArmBaseMotorState;
    }
    public CANSparkMax getArmBaseMotor(){
        return this.m_armBaseMotor.get();
    }

    public void periodic(){
        
    }

}