package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmPivot extends SubsystemBase {

    public void periodic() {}

     public enum MotorState {
        ON,
        OFF,
        REVERSED
    }

    // Motor Initialization
    public CANSparkMax pivotMotor = new CANSparkMax(Constants.ArmPivot.PivotMotor, MotorType.kBrushless);
    
    // Starts motors in their off state
    public MotorState pivotMotorState = MotorState.OFF;

    // Motor sparkmax settings
    public ArmPivot() {
        this.pivotMotor.setIdleMode(IdleMode.kBrake);

        this.pivotMotor.setSmartCurrentLimit(51);
       
    }
    
    public void setPivotMotorState(MotorState state) {
        // set the current state
        this.pivotMotorState = state;
        
        // set motor state
        switch (state) {
            case ON:
                // On
                this.pivotMotor.set(Constants.ArmPivot.PivotMotorSpeedForwards);
                break;
            case OFF:
                // Off
                this.pivotMotor.set(0);
                
                break;
            case REVERSED:
                // Reversed
                this.pivotMotor.set(Constants.ArmPivot.PivotMotorSpeedBackwards);
                break;
            default:
                this.setPivotMotorState(MotorState.OFF);
        }
    }

    public double getPivotMotorPosition() {
        return this.pivotMotor.getEncoder().getPosition();
    }

    public void setPivotMotorPosition(double position) {
        this.pivotMotor.getEncoder().setPosition(position);
    }
}