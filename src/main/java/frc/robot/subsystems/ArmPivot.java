package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmBase.BaseMotorState;

public class ArmPivot extends SubsystemBase {

    
    public enum PivotMotorState {
        ON,
        OFF,
        REVERSED
    }
    
    // Motor Initialization
    public CANSparkMax pivotMotor = new CANSparkMax(Constants.ArmPivot.PivotMotor, MotorType.kBrushless);
    
    // Starts motors in their off state
    public PivotMotorState pivotMotorState = PivotMotorState.OFF;
    
    // //Through Bore Encoder
    // DutyCycleEncoder throughBoreEncoderPivot = new DutyCycleEncoder(0);

    // Motor sparkmax settings
    public ArmPivot() {
        this.pivotMotor.setIdleMode(IdleMode.kBrake);
        
        this.pivotMotor.setSmartCurrentLimit(51);
        
    }
    
    public void setPivotMotorState(PivotMotorState state) {
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
                this.setPivotMotorState(PivotMotorState.OFF);
        }
    }

    public double getPivotMotorPosition() {
        return this.pivotMotor.getEncoder().getPosition();
    }

    public double getPivotMotorSpeed() {
        return this.pivotMotor.get();
    }

    public PivotMotorState getPivotMotorState() {
        return this.pivotMotorState;
    }
    
    public void setPivotMotorPosition(double position) {
        this.pivotMotor.getEncoder().setPosition(position);
    }

    public void periodic() {}
}