package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
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
    public CANSparkMax baseMotor = new CANSparkMax(Constants.ArmBase.BaseMotorCANID, MotorType.kBrushless);

    //Limit Switches
    public DigitalInput lowerGantryLimitSwitch = new DigitalInput(Constants.ArmBase.lowerLimitSwitchPort);
    public DigitalInput higherGantryLimitSwitch = new DigitalInput(Constants.ArmBase.higherLimitSwitchPort);

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
            if (higherGantryLimitSwitch.get() == false){
                this.baseMotor.set(Constants.ArmBase.SpeedForwards);
            }
            break;
            
            case OFF:
            // Off
            this.baseMotor.set(0);
            break;
            
            case REVERSED:
            // Reversed
                if (lowerGantryLimitSwitch.get() == false){
                    this.baseMotor.set(Constants.ArmBase.SpeedBackwards);
                }
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