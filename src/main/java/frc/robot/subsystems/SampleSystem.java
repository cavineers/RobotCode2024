package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class SampleSystem extends SubsystemBase {

    public void periodic(){

        // SmartDashboard.putNumber("AngleRotations", getArmChainMotorPosition());

    }
    // States for both motors
     public enum MotorState {
        ON,
        OFF,
        REVERSED
    }

    // Motor Initialization
    public CANSparkMax sampleMotor = new CANSparkMax(Constants.SampleSystem.SampleMotor, MotorType.kBrushless);
    
    // Starts motors in their off state
    public MotorState sampleMotorState = MotorState.OFF;

    // Motor sparkmax settings
    public SampleSystem() {
        this.sampleMotor.setIdleMode(IdleMode.kBrake);

        this.sampleMotor.setInverted(false);

        this.sampleMotor.setSmartCurrentLimit(51);
       
    }
    
    public void setSampleMotorState(MotorState state) {
        // set the current state
        this.sampleMotorState = state;
        
        // set motor state
        switch (state) {
            case ON:
                // On
                this.sampleMotor.set(Constants.SampleSystem.SampleMotorForwardSpeed);
                break;
            case OFF:
                // Off
                this.sampleMotor.set(0);
                
                break;
            case REVERSED:
                // Reversed
                this.sampleMotor.set(Constants.SampleSystem.SampleMotorBackwardSpeed);
                break;
            default:
                this.setSampleMotorState(MotorState.OFF);
        }
    }
}