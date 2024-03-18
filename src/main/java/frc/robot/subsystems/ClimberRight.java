package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberRight extends SubsystemBase {

    PIDController rightClimberPID = new PIDController(Constants.Climber.ProportionalGain, Constants.Climber.IntegralTerm, Constants.Climber.DerivitiveTerm);

    // Initialize the climber motor
    public CANSparkMax rightClimberMotor = new CANSparkMax(Constants.CanIDs.RightClimberCanID, MotorType.kBrushless);

    // Initialize the limit switch
    public DigitalInput rightClimberLimitSwitch = new DigitalInput(Constants.DIO.RightClimberLimitSwitch);

    // Motor states
    public enum RightClimberMotorState {
        ON, 
        OFF, 
        REVERSED
    }

    // Initial Motor State
    public RightClimberMotorState rightClimberMotorState = RightClimberMotorState.OFF;

    private double motorSetpoint = 0;

    public ClimberRight() {
        this.rightClimberMotor.setIdleMode(IdleMode.kBrake);

        this.rightClimberMotor.setInverted(false);

        // Set the amp limit when specified - TBD
        this.rightClimberMotor.setSmartCurrentLimit(51);
    }

    public boolean getLimitSwitch() {
        boolean switched;
        switched = this.rightClimberLimitSwitch.get();
        return switched;
    }

    // Set the motor's position (given in rotations)
    public void setRightClimberMotorPosition(double position) {
        this.rightClimberMotor.getEncoder().setPosition(position);
    }

    // Set the motor's speed (value between -1 and 1)
    public void setRightClimberMotorSpeed(double speed) {
        this.rightClimberMotor.set(speed);
    }

    // Get motor position (value returned in number of rotations)
    public double getRightClimberMotorPosition() {
        return this.rightClimberMotor.getEncoder().getPosition();
    }

    public double getRightClimberMotorSetPoint() {
        return this.motorSetpoint;
    }

    // Get motor speed (value between -1 and 1)
    public double getRightClimberMotorSpeed() {
        return this.rightClimberMotor.get();
    }

    // Get the current state of the motor
    public RightClimberMotorState getRightClimberMotorState() {
        return this.rightClimberMotorState;
    }

    public CANSparkMax getRightClimberMotor() {
        return this.rightClimberMotor;
    }

    public double getRightClimberSetPoint() {
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
            rightClimberPID.setSetpoint(motorSetpoint);
            double speed = rightClimberPID.calculate(getRightClimberMotorPosition());
    
            rightClimberMotor.set(speed);
        }

        if (this.motorSetpoint < Constants.Climber.LowerClimberMaxRotations) {
            this.motorSetpoint = Constants.Climber.LowerClimberMaxRotations;
        }

        if(getLimitSwitch() == true) {
            setRightClimberMotorPosition(Constants.Climber.UpperClimberMaxRotations);
        }

        SmartDashboard.putNumber("rightClimberPos", getRightClimberMotorPosition());
        SmartDashboard.putNumber("rightClimberSetPoint", getRightClimberMotorSetPoint());
        SmartDashboard.putBoolean("rightClimberLimitSwitch", getLimitSwitch());

    }
}