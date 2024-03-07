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


    private double gantryHeight;

    private double motorSetpoint = 0;

    // Starts motors in their off state
    public BaseMotorState baseMotorState = BaseMotorState.OFF;

    // Motor sparkmax settings
    public ArmBase() {
        this.baseMotor.setIdleMode(IdleMode.kCoast);

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

    public double getGantryHeightMeters() {
        
        gantryHeight = (Constants.ArmBase.dHeight * (motorSetpoint/Constants.ArmBase.dRotations)) + Constants.ArmBase.minGantryHeightMeters;
        
        return gantryHeight;
    }

    /** 
        @return double[] {min, max} minimum and maximum gantry heights for the current region
    */
    // public double[] getRegionRotationLimits(){
    //     double position = getBaseMotorPosition();

    //     if(position >= Constants.ArmBase.ArmPivotRegionGround[0] && position <= Constants.ArmBase.ArmPivotRegionGround[1]) {
    //         SmartDashboard.putString("Region", "Ground");
    //         return new double[]{Constants.ArmPivot.ArmPivotRotationGround[0], Constants.ArmPivot.ArmPivotRotationGround[1]};
    //     } else if(position >= Constants.ArmBase.ArmPivotRegionSwerve[0] && position <= Constants.ArmBase.ArmPivotRegionSwerve[1]) {
    //         SmartDashboard.putString("Region", "Swerve");
    //         return new double[]{Constants.ArmPivot.ArmPivotRotationSwerve[0], Constants.ArmPivot.ArmPivotRotationSwerve[1]};
    //     } else if(position >= Constants.ArmBase.ArmPivotRegionMidGantry[0] && position <= Constants.ArmBase.ArmPivotRegionMidGantry[1]) {
    //         SmartDashboard.putString("Region", "MidGantry");
    //         return new double[]{Constants.ArmPivot.ArmPivotRotationMidGantry[0], Constants.ArmPivot.ArmPivotRotationMidGantry[1]};
    //     } else {
    //         SmartDashboard.putString("Region", "UpperGantry");
    //         return new double[]{Constants.ArmPivot.ArmPivotRotationUpperGantry[0], Constants.ArmPivot.ArmPivotRotationUpperGantry[1]};
    //     }
    // }

    public void periodic() {
    
        
        // clip the setpoint            
        if (motorSetpoint > Constants.ArmBase.MaxRotations) {
            motorSetpoint = Constants.ArmBase.MaxRotations;
        } else if (motorSetpoint < Constants.ArmBase.MinRotations) {
            motorSetpoint = Constants.ArmBase.MinRotations;
        }

        SmartDashboard.putNumber("GantryRot", getBaseMotorPosition());
        SmartDashboard.putNumber("Gantry SETPOINT", motorSetpoint);
        basePid.setSetpoint(motorSetpoint);
        double speed = basePid.calculate(getBaseMotorPosition());
        SmartDashboard.putNumber("Speed", speed);

        SmartDashboard.putBoolean("hIGHER", getGantryHigherLimitSwitch());
        SmartDashboard.putBoolean("lOWER", getGantryLowerLimitSwitch());
        
        baseMotor.set(speed);
        
        // LIMIT SWITCHES
        if (getGantryHigherLimitSwitch()) {
            setBaseMotorPosition(Constants.ArmBase.MaxRotations);
            this.motorSetpoint = Constants.ArmBase.MaxRotations;

        } else if (getGantryLowerLimitSwitch()) {
            setBaseMotorPosition(Constants.ArmBase.MinRotations);
            this.motorSetpoint = Constants.ArmBase.MinRotations;
        }
    }
}

