package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmBase extends SubsystemBase {

    public void periodic(){

    }

// Motor State for Arm Base
    public enum ArmBaseMotorState() {
        ON,
        OFF,
        REVERSED
    }

// Starts Motor
    public CANSparkMax m_armBaseMotor = new CANSparkMax(Constants.Arm.ArmBaseMotor, MotorType.kBrushless)

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
        return this.m_armBaseMotor;
    }

}