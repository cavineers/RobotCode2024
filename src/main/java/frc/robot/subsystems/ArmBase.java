package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmBase extends SubsystemBase {

    PIDController basePid = new PIDController(Constants.ArmBase.ProportionalGain, Constants.ArmBase.IntegralTerm, Constants.ArmBase.DerivitiveTerm);

    public enum BaseMotorState {
        ON, 
        OFF, 
        REVERSED
    }

    // Motor Initialization
    public CANSparkMax baseMotor = new CANSparkMax(Constants.CanIDs.GantryCANID, MotorType.kBrushless);

    // Limit Switches
    public DigitalInput lowerGantryLimitSwitch = new DigitalInput(Constants.DIO.GantryLowerLimitSwitch);
    public DigitalInput higherGantryLimitSwitch = new DigitalInput(Constants.DIO.GantryHigherLimitSwitch);

    private double motorSetpoint = 0;

    // Starts motors in their off state
    public BaseMotorState baseMotorState = BaseMotorState.OFF;

    // Motor sparkmax settings
    public ArmBase() {
        this.baseMotor.setIdleMode(IdleMode.kBrake);

        this.baseMotor.setSmartCurrentLimit(51);
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

    public void setSetpointAdd(double s){
        motorSetpoint += s;
        if(this.motorSetpoint > Constants.ArmBase.MaxRotations){
            this.motorSetpoint = Constants.ArmBase.MaxRotations;
        }else if(this.motorSetpoint < Constants.ArmBase.MinRotations){
            this.motorSetpoint = Constants.ArmBase.MinRotations;
        }
        
    }

    public void setSetpoint(double s){
        motorSetpoint = s;
        if(this.motorSetpoint > Constants.ArmBase.MaxRotations){
            this.motorSetpoint = Constants.ArmBase.MaxRotations;
        }else if(this.motorSetpoint < Constants.ArmBase.MinRotations){
            this.motorSetpoint = Constants.ArmBase.MinRotations;
        }
        
    }

    public void periodic() {
        SmartDashboard.putNumber("GantryRot", getBaseMotorPosition());

        if (this.motorSetpoint <= Constants.ArmBase.MaxRotations && this.motorSetpoint >= Constants.ArmBase.MinRotations){
            basePid.setSetpoint(motorSetpoint);
            double speed = basePid.calculate(getBaseMotorPosition());
            SmartDashboard.putNumber("Speed", speed);
    
            baseMotor.set(speed);
        }
    }

}