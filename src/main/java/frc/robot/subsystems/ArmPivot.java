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

    private PIDController pivotPid = new PIDController(Constants.ArmPivot.ProportionalGain, Constants.ArmPivot.IntegralTerm, Constants.ArmPivot.DerivitiveTerm);

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

        this.pivotPid.setTolerance(0.03);

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

        requiredSetpoint = ((Constants.ArmPivot.dRotations * (angle - Constants.ArmPivot.armPivotMinAngleDegrees)) / Constants.ArmPivot.dAngle) + Constants.ArmPivot.PivotMotorLowerRotationLimit;
        setSetpoint(requiredSetpoint);

        SmartDashboard.putNumber("Set Setpoint", requiredSetpoint);

    }

    public double getArmPivotAngle() {

        currentArmPivotAngle = ((motorSetpoint* Constants.ArmPivot.dAngle) / Constants.ArmPivot.dRotations);

        return currentArmPivotAngle;
    }

    public boolean isAtSetpoint(){
        return this.pivotPid.atSetpoint();
    }

    public void periodic() {

        // Clip setpoints
        double minRotation;
        double maxRotation = Constants.ArmPivot.PivotMotorUpperRotationLimit;

        if (armBase.getBaseMotorPosition() > Constants.ArmBase.PivotRegionRestMin) {
            minRotation = Constants.ArmPivot.PivotRestMinRotations;
        } else if (armBase.getBaseMotorPosition() < Constants.ArmBase.PivotRegionGroundMax) {
            minRotation = Constants.ArmPivot.PivotGroundMinRotations;
        } else { // between 1 rotation and 149 rotations
            minRotation = Constants.ArmPivot.PivotNormalMinRotations;
        }

        this.motorSetpoint = Math.max(minRotation, Math.min(maxRotation, this.motorSetpoint));
        currentMinimumRot = minRotation;

        // Failsafe
        if (getPivotAbsolute()<.3) {
            this.motorSetpoint = getPivotAbsolute();
        } else if (getPivotAbsolute()>.8) {
            this.motorSetpoint = getPivotAbsolute();
        }

        // Set motor speed
        pivotPid.setSetpoint(motorSetpoint);
        double speed = pivotPid.calculate(getPivotAbsolute());
        SmartDashboard.putNumber("PivotSpeed", speed);
        if (speed < -.1) {
            speed = -.1;
        }
        pivotMotor.set(speed);

        SmartDashboard.putNumber("PivotRot", getPivotAbsolute());
        SmartDashboard.putNumber("PIVOT SETPOINT", motorSetpoint);
        SmartDashboard.putNumber("PivotMin", currentMinimumRot);
    }

    public boolean atSetpoint() {
        return this.pivotPid.atSetpoint();
    }
}