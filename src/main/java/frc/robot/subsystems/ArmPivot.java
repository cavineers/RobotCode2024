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

    private double currentMinimumRot;
    
    // Motor sparkmax settings
    public ArmPivot(ArmBase armBase) {
        this.pivotMotor.setIdleMode(IdleMode.kBrake);
        this.pivotMotor.setSmartCurrentLimit(80);

        this.pivotMotor.setInverted(true);
        this.pivotPid.setTolerance(Constants.ArmPivot.PivotSetpointTolerance);

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
        motorSetpoint += s;
    }

    public void setSetpoint(double s){
        motorSetpoint = s;
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

        
        // SmartDashboard.putNumber("Pivot Limit Lower", limits[0]);
        // SmartDashboard.putNumber("Pivot Limit Upper", limits[1]);
        // Clip setpoints
        if(armBase.getBaseMotorPosition() > Constants.ArmBase.PivotRegionRestMin) {
            if (this.motorSetpoint > Constants.ArmPivot.PivotMotorUpperRotationLimit) {
                this.motorSetpoint = Constants.ArmPivot.PivotMotorUpperRotationLimit;
            } else if (this.motorSetpoint < Constants.ArmPivot.PivotRestMinRotations) {
                this.motorSetpoint = Constants.ArmPivot.PivotRestMinRotations;
                currentMinimumRot = Constants.ArmPivot.PivotRestMinRotations;
            }
        } else if(armBase.getBaseMotorPosition() < Constants.ArmBase.PivotRegionGroundMax) {
            if (this.motorSetpoint > Constants.ArmPivot.PivotMotorUpperRotationLimit) {
                this.motorSetpoint = Constants.ArmPivot.PivotMotorUpperRotationLimit;
            } else if (this.motorSetpoint < Constants.ArmPivot.PivotGroundMinRotations) {
                this.motorSetpoint = Constants.ArmPivot.PivotGroundMinRotations;
                currentMinimumRot = Constants.ArmPivot.PivotGroundMinRotations;
            }
        } else {
            if (this.motorSetpoint > Constants.ArmPivot.PivotMotorUpperRotationLimit) {
                this.motorSetpoint = Constants.ArmPivot.PivotMotorUpperRotationLimit;
            } else if (this.motorSetpoint < Constants.ArmPivot.PivotNormalMinRotations) {
                this.motorSetpoint = Constants.ArmPivot.PivotNormalMinRotations;
                currentMinimumRot = Constants.ArmPivot.PivotNormalMinRotations;
            }
        }

        SmartDashboard.putNumber("PivotRot", getPivotAbsolute());
        SmartDashboard.putNumber("PIVOT SETPOINT", motorSetpoint);
        // Set motor speed
        pivotPid.setSetpoint(motorSetpoint);
        double speed = pivotPid.calculate(getPivotAbsolute());
        SmartDashboard.putNumber("Speed", speed);
        pivotMotor.set(speed);
        SmartDashboard.putNumber("Setpoint", this.motorSetpoint);
        SmartDashboard.putNumber("PivotMin", currentMinimumRot);
    }

    public boolean atSetpoint() {
        return this.pivotPid.atSetpoint();
    }
}