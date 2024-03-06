package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class ArmPivot extends SubsystemBase {

    PIDController pivotPid = new PIDController(Constants.ArmPivot.ProportionalGain, Constants.ArmPivot.IntegralTerm, Constants.ArmPivot.DerivitiveTerm);

    public enum PivotMotorState {
        ON,
        OFF, 
        REVERSED
    }

    private ArmBase armBase;

    // Motor Initialization
    public CANSparkMax pivotMotor = new CANSparkMax(Constants.CanIDs.PivotCanID, MotorType.kBrushless);

    // Starts motors in their off state
    public PivotMotorState pivotMotorState = PivotMotorState.OFF;

    // Through Bore Encoder
    public DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(Constants.DIO.ArmBoreEncoder);

    private double motorSetpoint;

    private double currentArmPivotAngle;
    private double requiredSetpoint;
    
    // Motor sparkmax settings
    public ArmPivot(ArmBase armBase) {
        this.pivotMotor.setIdleMode(IdleMode.kBrake);
        this.pivotMotor.setSmartCurrentLimit(51);
        this.pivotMotor.setInverted(true);
        this.motorSetpoint = pivotEncoder.getAbsolutePosition();
        this.armBase = armBase;
    }

    public void initializeDutyEncoder(){
        this.motorSetpoint = pivotEncoder.getAbsolutePosition();
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

    public double getPivotAbsolute(){
        return this.pivotEncoder.getAbsolutePosition();
    }

    public double getPivotEncoderFrequency() {
        return this.pivotEncoder.getFrequency();
    }

    public void setSetpointAdd(double s){
        if((this.motorSetpoint += s) > Constants.ArmPivot.PivotMotorUpperRotationLimit){
            this.motorSetpoint = Constants.ArmPivot.PivotMotorUpperRotationLimit;
          
        }else if((this.motorSetpoint += s) < Constants.ArmPivot.PivotMotorLowerRotationLimit){
            this.motorSetpoint = Constants.ArmPivot.PivotMotorLowerRotationLimit;
      
      
        }else{
            motorSetpoint += s;
        }
        
    }

    public void setSetpoint(double s){
        motorSetpoint = s;
        if(s > Constants.ArmPivot.PivotMotorUpperRotationLimit){
            this.motorSetpoint = Constants.ArmPivot.PivotMotorUpperRotationLimit;
        }else if(s < Constants.ArmPivot.PivotMotorLowerRotationLimit){
            this.motorSetpoint = Constants.ArmPivot.PivotMotorLowerRotationLimit;
        }else{
            motorSetpoint = s;
        }
        
    }

    public void setArmPivotAngle(Double angle) {

        requiredSetpoint = (angle * Constants.ArmPivot.dRotations) / Constants.ArmPivot.dAngle;
        setSetpoint(requiredSetpoint);

    }

    public double getArmPivotAngle() {

        currentArmPivotAngle = ((motorSetpoint* Constants.ArmPivot.dAngle) / Constants.ArmPivot.dRotations);

        return currentArmPivotAngle;
    }

    public double getArmPivotHypToBaseline() {
        return (getArmPivotAngle() - Constants.ArmPivot.armPivotTriangleAngleFromPivotDegrees);
    }

    public void periodic() {

        SmartDashboard.putNumber("PivotRot", getPivotAbsolute());
        SmartDashboard.putNumber("PIVOT SETPOINT", motorSetpoint);
        
        double[] limits = this.armBase.getRegionRotationLimits();
        // Clip setpoints
        if (this.motorSetpoint > limits[1]) {
            this.motorSetpoint = limits[1];
        } else if (this.motorSetpoint < limits[0]) {
            this.motorSetpoint = limits[0];
        }

        // Set motor speed
        pivotPid.setSetpoint(motorSetpoint);
        double speed = pivotPid.calculate(getPivotAbsolute());
        pivotMotor.set(speed);
        SmartDashboard.putNumber("Setpoint", this.motorSetpoint);
    }
}