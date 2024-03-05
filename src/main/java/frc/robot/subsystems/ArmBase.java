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
    
    public boolean getLeftGantryUpperSwitch() {
        return this.leftGantryHigherLimitSwitch.get();
    }

    public boolean getLeftGantryLowerSwitch() {
        return this.leftGantryLowerLimitSwitch.get();
    }

    public boolean getRightGantryUpperSwitch() {
        return this.rightGantryHigherLimitSwitch.get();
    }

    public boolean getRightGantryLowerSwitch() {
        return this.rightGantryLowerLimitSwitch.get();
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
    public double[] getRegionRotationLimits(){
        double position = getBaseMotorPosition();

        if(position >= Constants.ArmBase.ArmPivotRegionGround[0] && position <= Constants.ArmBase.ArmPivotRegionGround[1])
            return new double[]{Constants.ArmPivot.ArmPivotRotationGround[0], Constants.ArmPivot.ArmPivotRotationGround[1]};
        else if(position >= Constants.ArmBase.ArmPivotRegionSwerve[0] && position <= Constants.ArmBase.ArmPivotRegionSwerve[1])
            return new double[]{Constants.ArmPivot.ArmPivotRotationSwerve[0], Constants.ArmPivot.ArmPivotRotationSwerve[1]};
        else if(position >= Constants.ArmBase.ArmPivotRegionMidGantry[0] && position <= Constants.ArmBase.ArmPivotRegionMidGantry[1])
            return new double[]{Constants.ArmPivot.ArmPivotRotationMidGantry[0], Constants.ArmPivot.ArmPivotRotationMidGantry[1]};
        else (position >= Constants.ArmBase.ArmPivotRegionUpperGantry[0] && position <= Constants.ArmBase.ArmPivotRegionUpperGantry[1])
            return new double[]{Constants.ArmPivot.ArmPivotRotationUpperGantry[0], Constants.ArmPivot.ArmPivotRotationUpperGantry[1]};
    }

    public void periodic() {
        SmartDashboard.putNumber("GantryRot", getBaseMotorPosition());
        SmartDashboard.putNumber("Gantry SETPOINT", motorSetpoint);
            
        if (motorSetpoint > Constants.ArmBase.MaxRotations) {
            motorSetpoint = Constants.ArmBase.MaxRotations;
        }else if (motorSetpoint < Constants.ArmBase.MinRotations) {
            motorSetpoint = Constants.ArmBase.MinRotations;
        }
        double speed = basePid.calculate(getBaseMotorPosition());
        SmartDashboard.putNumber("Speed", speed);
        
        baseMotor.set(speed);
        


        // LIMIT SWITCHES
        if (getLeftGantryUpperSwitch() && getRightGantryUpperSwitch()) {
            setBaseMotorPosition(Constants.ArmBase.MaxRotations);
            this.motorSetpoint = Constants.ArmBase.MaxRotations;

        } else if (getLeftGantryLowerSwitch() && getRightGantryLowerSwitch()) {
            setBaseMotorPosition(Constants.ArmBase.MinRotations);
            this.motorSetpoint = Constants.ArmBase.MinRotations;
        }
    }
}

