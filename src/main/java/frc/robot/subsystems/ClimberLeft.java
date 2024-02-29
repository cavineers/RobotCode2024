package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberLeft extends SubsystemBase {

    PIDController leftClimberPID = new PIDController(Constants.Climber.ProportionalGain, Constants.Climber.IntegralTerm, Constants.Climber.DerivitiveTerm);

    // Initialize the climber motor
    public CANSparkMax leftClimberMotor = new CANSparkMax(Constants.CanIDs.LeftClimberCanID, MotorType.kBrushless);

    public DigitalInput leftClimberTopLimitSwitch = new DigitalInput(Constants.DIO.LeftClimberTopLimitSwitch);

    // Motor states
    public enum LeftClimberMotorState {
        ON, 
        OFF, 
        REVERSED
    }

    // Initial Motor State
    public LeftClimberMotorState leftClimberMotorState = LeftClimberMotorState.OFF;

    private double motorSetpoint = 0;

    public ClimberLeft() {
        this.leftClimberMotor.setIdleMode(IdleMode.kBrake);

        this.leftClimberMotor.setInverted(false);

        // Set the amp limit when specified - TBD
        this.leftClimberMotor.setSmartCurrentLimit(51);
    }

    public boolean getLimitSwitch() {
        boolean switched;
        switched = this.leftClimberTopLimitSwitch.get();
        return switched;
    }

    // Set the motor's position (given in rotations)
    public void setLeftClimberMotorPosition(double position) {
        this.leftClimberMotor.getEncoder().setPosition(position);
    }

    // Set the motor's speed (value between -1 and 1)
    public void setLeftClimberMotorSpeed(double speed) {
        this.leftClimberMotor.set(speed);
    }

    // Get motor position (value returned in number of rotations)
    public double getLeftClimberMotorPosition() {
        return this.leftClimberMotor.getEncoder().getPosition();
    }

    public double getLeftClimberMotorSetPoint() {
        return this.motorSetpoint;
    }

    // Get motor speed (value between -1 and 1)
    public double getLeftClimberMotorSpeed() {
        return this.leftClimberMotor.get();
    }

    // Get the current state of the motor
    public LeftClimberMotorState getLeftClimberMotorState() {
        return this.leftClimberMotorState;
    }

    public CANSparkMax getLeftClimberMotor() {
        return this.leftClimberMotor;
    }

    public double getLeftClimberSetPoint() {
        return this.motorSetpoint;
    }

    public void setSetpointAdd(double s){
        motorSetpoint += s;
        if(this.motorSetpoint > Constants.Climber.UpperClimberMaxRotations){
            this.motorSetpoint = Constants.Climber.UpperClimberMaxRotations;
        }else if(this.motorSetpoint < Constants.Climber.LowerClimberMaxRotations){
            this.motorSetpoint = Constants.Climber.LowerClimberMaxRotations;
        }
        
    }

    public void setSetpoint(double s){
        motorSetpoint = s;
        if(this.motorSetpoint > Constants.Climber.UpperClimberMaxRotations){
            this.motorSetpoint = Constants.Climber.UpperClimberMaxRotations;
        }else if(this.motorSetpoint < Constants.Climber.LowerClimberMaxRotations){
            this.motorSetpoint = Constants.Climber.LowerClimberMaxRotations;
        }
    }

    @Override
    public void periodic() {

        if (this.motorSetpoint <= Constants.Climber.UpperClimberMaxRotations && this.motorSetpoint >= Constants.Climber.LowerClimberMaxRotations){
            leftClimberPID.setSetpoint(motorSetpoint);
            double speed = leftClimberPID.calculate(getLeftClimberMotorPosition());
            SmartDashboard.putNumber("Speed", speed);
    
            leftClimberMotor.set(speed);
        }

        if(getLimitSwitch() == true) {
            setLeftClimberMotorPosition(Constants.Climber.LowerClimberMaxRotations);
        }

        SmartDashboard.putNumber("leftClimberPos", getLeftClimberMotorPosition());
        SmartDashboard.putNumber("leftClimberSetPoint", getLeftClimberMotorSetPoint());
        SmartDashboard.putBoolean("leftClimberLimitSwitch", getLimitSwitch());
    }
}