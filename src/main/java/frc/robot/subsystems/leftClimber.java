package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LeftClimber extends SubsystemBase {

    public CANSparkMax leftClimberMotor = new CANSparkMax(Constants.Climber.leftClimberMotor, MotorType.kBrushless);
  
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

        //Set the amp limit when specified - this needs to be changed
        this.leftClimberMotor.setSmartCurrentLimit(51);
    }

    //Allow for changing motor states
    public void setLeftClimberMotorState(LeftClimberMotorState state) {

        this.leftClimberMotorState = state;

        //If none of the states have been specified, set the motor state to OFF by default
        if (state != LeftClimberMotorState.OFF && state != LeftClimberMotorState.ON && state != LeftClimberMotorState.REVERSED) {
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

    public void periodic() {
        if (this.leftClimberMotorState == leftClimberMotorState.ON) {
         
        }

        if (this.leftClimberMotorState == leftClimberMotorState.OFF) {
         
        }

        if (this.leftClimberMotorState == leftClimberMotorState.REVERSED) {
         
        }
    }
}