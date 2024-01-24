package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmPivot extends SubsystemBase {

// Motor State for Arm Pivot
    public enum ArmPivotMotorState() {
        ON,
        OFF,
        REVERSED
    }

// Starts Motor
    public CANSparkMax m_armPivotMotor = new CANSparkMax(Constants.Arm.ArmPivotMotor, MotorType.kBrushless);

    public ArmPivotMotorState m_armPivotMotorState = ArmPivotMotorState.OFF;

    public ArmPivot(){
        this.ArmPivotMotor.setIdleMode(IdleMode.kBrake);
        this.ArmPivotMotor.setSmartCurrentLimit(41); ///TBD
    }

    public void setArmPivotMotorState(ArmPivotMotorState state){
        this.ArmPivotMotor = state;
        switch(state){
            case ON:
                this.ArmPivotMotor.set(Constants.Arm.ArmPivotForwardSpeed);
                break;
            case OFF: 
                this.ArmPivotMotor.set(speed: 0.0);
            case REVERSED: 
                this.ArmPivotMotor.set(Constants.Arm.ArmPivotBackwardsSpeed);
                break;
            default: 
                this.setArmPivotMotorState(ArmPivotMotorState.OFF);
        }
    }

    public ArmPivotMotorState getArmPivotMotorState(){
        return this.ArmPivotMotorState;
    }
    public CANSparkMax getArmPivotMotor(){
        return this.m_armPivotMotor.get();
    }

    public void periodic(){
        
    }

}