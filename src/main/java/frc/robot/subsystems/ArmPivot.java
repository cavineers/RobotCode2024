package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmBase.BaseMotorState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

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
    
    //Through Bore Encoder
    //DutyCycleEncoder pivotThroughBoreEncoder = new DutyCycleEncoder(1);

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
/*
    public void setDistancePerRotation(double distance){
        this.pivotThroughBoreEncoder.setDistancePerRotation(1);
    }

    public void getDistance(double distance){
        this.pivotThroughBoreEncoder.getDistance();
    }

    public void isConnected(boolean connection){
        this.pivotThroughBoreEncoder.isConnected();
    }

    public void reset(boolean reset){
        this.pivotThroughBoreEncoder.reset();
    }

    public void getPositionOffset(double positionoffset){
        this.pivotThroughBoreEncoder.getPositionOffset();
    }

    public void setPositionOffset(double positionoffset){
        this.pivotThroughBoreEncoder.setPositionOffset(positionoffset);
    }
*/
    public void periodic() {}
}