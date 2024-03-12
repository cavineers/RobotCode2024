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
    public DigitalInput GantryLowerLimitSwitch = new DigitalInput(Constants.DIO.GantryLowerLimitSwitch);
    public DigitalInput GantryHigherLimitSwitch = new DigitalInput(Constants.DIO.GantryHigherLimitSwitch);

    private double motorSetpoint = 0;

    // Starts motors in their off state
    public BaseMotorState baseMotorState = BaseMotorState.OFF;

    // Motor sparkmax settings
    public ArmBase() {
        this.baseMotor.setIdleMode(IdleMode.kCoast);

        this.baseMotor.setSmartCurrentLimit(80);
        this.basePid.setTolerance(Constants.ArmBase.BaseSetpointTolerance);
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
    
    public boolean getGantryHigherLimitSwitch() {
        return this.GantryHigherLimitSwitch.get();
    }

    public boolean getGantryLowerLimitSwitch() {
        return this.GantryLowerLimitSwitch.get();
    }

    public void setSetpointAdd(double s){
       this.motorSetpoint += s;
        
    }

    public void setSetpoint(double s){
       
        this.motorSetpoint = s;
    
    }

    public void periodic() {
    
        
        // clip the setpoint            
        if (motorSetpoint > Constants.ArmBase.MaxRotations) {
            motorSetpoint = Constants.ArmBase.MaxRotations;
        } else if (motorSetpoint < Constants.ArmBase.MinRotations) {
            motorSetpoint = Constants.ArmBase.MinRotations;
        }

        basePid.setSetpoint(motorSetpoint);
        double speed = basePid.calculate(getBaseMotorPosition());
        
        SmartDashboard.putBoolean("HigherGantrySwitch", getGantryHigherLimitSwitch());
        SmartDashboard.putBoolean("LowerGantrySwitch", getGantryLowerLimitSwitch());
        SmartDashboard.putNumber("GantryRot", getBaseMotorPosition());
        SmartDashboard.putNumber("Gantry SETPOINT", motorSetpoint);
        
        baseMotor.set(speed);
        
        // LIMIT SWITCHES
        if (getGantryHigherLimitSwitch()) {
            setBaseMotorPosition(Constants.ArmBase.MaxRotations);

        } else if (getGantryLowerLimitSwitch()) {
            setBaseMotorPosition(Constants.ArmBase.MinRotations);
        }
    }

    public boolean atSetpoint() {
        return this.basePid.atSetpoint();
    }
}

