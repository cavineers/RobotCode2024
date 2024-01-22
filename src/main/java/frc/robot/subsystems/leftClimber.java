package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LeftClimber extends SubsystemBase {

    //Initialize the climber motor
    public CANSparkMax leftClimberMotor = new CANSparkMax(Constants.Climber.LeftClimberMotor, MotorType.kBrushless);

    //Initialize the limit switch
    private DigitalInput leftClimberLimitSwitch = new DigitalInput(Constants.DIO.LeftClimberLimitSwitch);
  
    //Motor states
    public enum LeftClimberMotorState {
        ON,
        OFF,
        REVERSED
    }

    //Initial Motor State
    public LeftClimberMotorState leftClimberMotorState = LeftClimberMotorState.OFF;

    public LeftClimber() {
        this.leftClimberMotor.setIdleMode(IdleMode.kBrake);

        this.leftClimberMotor.setInverted(false);

        //Set the amp limit when specified - TBD
        this.leftClimberMotor.setSmartCurrentLimit(51);
    }

    //Allow for changing motor states
    public void setLeftClimberMotorState(LeftClimberMotorState state) {

        //Set the current state
        this.leftClimberMotorState = state;
            
        switch (state) {
            case ON:
                //On: Set the extension speed of the climber
                this.leftClimberMotor.set(Constants.Climber.ClimberExtensionSpeed);
                break;
            case OFF:
                //Off: Set the speed to zero
                this.leftClimberMotor.set(0.0);
                break;
            case REVERSED:
                //Reversed: Set the reversal speed of the climber
                this.leftClimberMotor.set(Constants.Climber.ClimberExtensionSpeedRev);
                break;
            default:
                this.setLeftClimberMotorState(LeftClimberMotorState.OFF);
        }
    }

    //Set the motor's position (given in rotations)
    public void setLeftClimberMotorPosition(double position) {
        this.leftClimberMotor.getEncoder().setPosition(position);
    }

    //Set the motor's speed (value between -1 and 1)
    public void setLeftClimberMotorSpeed(double speed) {
        this.leftClimberMotor.set(speed);
    }

     //Get motor position (value returned in number of rotations)
    public double getLeftClimberMotorPosition() {
        return this.leftClimberMotor.getEncoder().getPosition();
    }
    
    //Get motor speed (value between -1 and 1)
    public double getLeftClimberMotorSpeed() {
        return this.leftClimberMotor.get();
    }

    //Get the current state of the motor
    public LeftClimberMotorState getLeftClimberMotorState() {
        return this.leftClimberMotorState;
    }

    public void periodic(){

        //Turn off the motor once it has hit the limit switch
        if (this.leftClimberLimitSwitch.get() == true) {
            leftClimberMotor.leftClimberMotorState = LeftClimberMotorState.OFF;
        }
    }
}