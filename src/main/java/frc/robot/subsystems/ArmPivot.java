package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmPivot extends SubsystemBase {

    PIDController pivotPid = new PIDController(Constants.ArmPivot.ProportionalGain, Constants.ArmPivot.IntegralTerm, Constants.ArmPivot.DerivitiveTerm);

    public enum PivotMotorState {
        ON,
        OFF, 
        REVERSED
    }

    // Motor Initialization
    public CANSparkMax pivotMotor = new CANSparkMax(Constants.CanIDs.PivotCanID, MotorType.kBrushless);

    // Starts motors in their off state
    public PivotMotorState pivotMotorState = PivotMotorState.OFF;

    // Through Bore Encoder
    public DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(Constants.DIO.ArmBoreEncoder);

    private double motorSetpoint = 0;

    private double currentArmPivotAngle;

    // Motor sparkmax settings
    public ArmPivot() {
        this.pivotMotor.setIdleMode(IdleMode.kBrake);
        this.pivotMotor.setSmartCurrentLimit(51);
        this.pivotMotor.setInverted(true);
    }

    public double getPivotMotorPosition() {
        return this.pivotMotor.getEncoder().getPosition();
    }

    public double getPivotMotorSpeed() {
        return this.pivotMotor.get();
    }

    public PivotMotorState getPivotMotorState() {
        return this.pivotMotorState;
    }

    public void setPivotMotorPosition(double position) {
        this.pivotMotor.getEncoder().setPosition(position);
    }

    public double getPivotEncoderPosition() {
        //return this.pivotEncoder.get();
        return this.pivotMotor.getEncoder().getPosition();
    }

    public double getPivotEncoderFrequency() {
        return this.pivotEncoder.getFrequency();
    }

    public void setSetpointAdd(double s){
        motorSetpoint += s;
        if(this.motorSetpoint > Constants.ArmPivot.PivotMotorUpperRotationLimit){
            this.motorSetpoint = Constants.ArmPivot.PivotMotorUpperRotationLimit;
        }else if(this.motorSetpoint < Constants.ArmPivot.PivotMotorLowerRotationLimit){
            this.motorSetpoint = Constants.ArmPivot.PivotMotorLowerRotationLimit;
        }
        
    }

    public void setSetpoint(double s){
        motorSetpoint = s;
        if(this.motorSetpoint > Constants.ArmPivot.PivotMotorUpperRotationLimit){
            this.motorSetpoint = Constants.ArmPivot.PivotMotorUpperRotationLimit;
        }else if(this.motorSetpoint < Constants.ArmPivot.PivotMotorLowerRotationLimit){
            this.motorSetpoint = Constants.ArmPivot.PivotMotorLowerRotationLimit;
        }
        
    }

    public void setArmPivotAngle(Double angle) {

        motorSetpoint = (angle * Constants.ArmPivot.dRotations) / Constants.ArmPivot.dAngle;

    }

    public double getArmPivotAngle() {
        currentArmPivotAngle = (Constants.ArmBase.dHeight * (motorSetpoint/Constants.ArmBase.dRotations)) + Constants.ArmBase.minGantryHeightMeters;

        return currentArmPivotAngle;
    }

    public double getArmPivotHypToBaseline() {
        return (getArmPivotAngle() - Constants.ArmPivot.armPivotTriangleAngleFromPivotDegrees);
    }

    public void periodic() {

        SmartDashboard.putNumber("PivotRot", getPivotMotorPosition());
        SmartDashboard.putNumber("PivotHypToBaseline", getArmPivotHypToBaseline());
        
        if (this.motorSetpoint <= Constants.ArmPivot.PivotMotorUpperRotationLimit && this.motorSetpoint >= Constants.ArmPivot.PivotMotorLowerRotationLimit){
            pivotPid.setSetpoint(motorSetpoint);
            double speed = pivotPid.calculate(getPivotMotorPosition());
            SmartDashboard.putNumber("Speed", speed);
    
            pivotMotor.set(speed);
        }
        SmartDashboard.putNumber("Setpoint", this.motorSetpoint);
    }
}