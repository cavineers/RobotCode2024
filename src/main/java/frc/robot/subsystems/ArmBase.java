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
    public DigitalInput leftGantryLowerLimitSwitch = new DigitalInput(Constants.DIO.LeftGantryLowerLimitSwitch);
    public DigitalInput leftGantryHigherLimitSwitch = new DigitalInput(Constants.DIO.LeftGantryHigherLimitSwitch);
    public DigitalInput rightGantryLowerLimitSwitch = new DigitalInput(Constants.DIO.RightGantryLowerLimitSwitch);
    public DigitalInput rightGantryHigherLimitSwitch = new DigitalInput(Constants.DIO.RightGantryHigherLimitSwitch);


    private double gantryHeight;

    private double motorSetpoint = 0;

    // Starts motors in their off state
    public BaseMotorState baseMotorState = BaseMotorState.OFF;

    // Motor sparkmax settings
    public ArmBase() {
        this.baseMotor.setIdleMode(IdleMode.kBrake);

        this.baseMotor.setSmartCurrentLimit(80);
    }

    public void initializeEncoder(){
        this.motorSetpoint = baseMotor.getEncoder().getPosition();
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
        if((this.motorSetpoint += s) > Constants.ArmBase.MaxRotations){
            this.motorSetpoint = Constants.ArmBase.MaxRotations;
        }else if((this.motorSetpoint += s)< Constants.ArmBase.MinRotations){
            this.motorSetpoint = Constants.ArmBase.MinRotations;
        }else{
            this.motorSetpoint += s;
        }
        
    }

    public void setSetpoint(double s){
       
        if(s > Constants.ArmBase.MaxRotations){
            this.motorSetpoint = Constants.ArmBase.MaxRotations;
        }else if(s < Constants.ArmBase.MinRotations){
            this.motorSetpoint = Constants.ArmBase.MinRotations;
        }else{
            this.motorSetpoint = s;
        }
        
    }

    public double getGantryHeightMeters() {
        
        gantryHeight = (Constants.ArmBase.dHeight * (motorSetpoint/Constants.ArmBase.dRotations)) + Constants.ArmBase.minGantryHeightMeters;
        
        return gantryHeight;
    }

    /** 
        @return double[] {min, max} minimum and maximum gantry heights for the current region
    */
    public double[] getRegionLimits(){
    
    }

    public void periodic() {
        SmartDashboard.putNumber("GantryRot", getBaseMotorPosition());
        SmartDashboard.putNumber("Gantry SETPOINT", motorSetpoint);


        
        basePid.setSetpoint(motorSetpoint);
        double speed = basePid.calculate(getBaseMotorPosition());
        SmartDashboard.putNumber("Speed", speed);
    
        baseMotor.set(speed);


        // LIMIT SWITCHES
        if (this.leftGantryHigherLimitSwitch.get() == true && this.rightGantryHigherLimitSwitch.get() == true) {
            setBaseMotorPosition(Constants.ArmBase.MaxRotations);
            this.motorSetpoint = Constants.ArmBase.MaxRotations;

        } else if (this.leftGantryLowerLimitSwitch.get() == true && this.rightGantryLowerLimitSwitch.get() == true) {
            setBaseMotorPosition(Constants.ArmBase.MinRotations);
            this.motorSetpoint = Constants.ArmBase.MinRotations;
        }
    }

}