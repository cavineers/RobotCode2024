package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RightClimber extends SubsystemBase {

    //Initialize the climber motor
    public CANSparkMax rightClimberMotor = new CANSparkMax(Constants.Climber.RightClimberMotor, MotorType.kBrushless);

    //Initialize the limit switch
    private DigitalInput rightClimberLimitSwitch = new DigitalInput(Constants.DIO.RightClimberLimitSwitch);
  
    //Motor states
    public enum RightClimberMotorState {
        ON,
        OFF,
        REVERSED
    }

    //Initial Motor State
    public RightClimberMotorState rightClimberMotorState = RightClimberMotorState.OFF;

    public RightClimber() {
        this.rightClimberMotor.setIdleMode(IdleMode.kBrake);

        this.rightClimberMotor.setInverted(false);

        //Set the amp limit when specified - TBD
        this.rightClimberMotor.setSmartCurrentLimit(51);
    }

    //Allow for changing motor states
    public void setRightClimberMotorState(RightClimberMotorState state) {

        //Set the current state
        this.rightClimberMotorState = state;
            
        switch (state) {
            case ON:
                System.out.println("right climber");
                //On: Set the extension speed of the climber
                this.rightClimberMotor.set(Constants.Climber.ClimberExtensionSpeed);
                break;
            case OFF:
                //Off: Set the speed to zero
                this.rightClimberMotor.set(0.0);
                break;
            case REVERSED:
                //Reversed: Set the reversal speed of the climber
                this.rightClimberMotor.set(Constants.Climber.ClimberExtensionSpeedRev);
                break;
            default:
                this.setRightClimberMotorState(RightClimberMotorState.OFF);
        }
    }

    //Getters and setters 

    public boolean getLimitSwitch() {
        return this.rightClimberLimitSwitch.get();
    }  

    //Set the motor's position (given in rotations)
    public void setRightClimberMotorPosition(double position) {
        this.rightClimberMotor.getEncoder().setPosition(position);
    }

    //Set the motor's speed (value between -1 and 1)
    public void setRightClimberMotorSpeed(double speed) {
        this.rightClimberMotor.set(speed);
    }

    //Get motor position (value returned in number of rotations)
    public double getRightClimberMotorPosition() {
        return this.rightClimberMotor.getEncoder().getPosition();
    }
    
    //Get motor speed (value between -1 and 1)
    public double getRightClimberMotorSpeed() {
        return this.rightClimberMotor.get();
    }

    //Get the current state of the motor
    public RightClimberMotorState getRightClimberMotorState() {
        return this.rightClimberMotorState;
    }

    public CANSparkMax getRightClimberMotor() {
        return this.rightClimberMotor;
    }
}