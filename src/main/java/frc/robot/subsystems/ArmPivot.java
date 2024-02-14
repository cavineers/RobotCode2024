package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmPivot extends SubsystemBase {

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

    // Motor sparkmax settings
    public ArmPivot() {
        this.pivotMotor.setIdleMode(IdleMode.kBrake);
        this.pivotMotor.setSmartCurrentLimit(51);
        this.pivotMotor.setInverted(false);

    }

    public void setPivotMotorState(PivotMotorState state) {
        // set the current state
        this.pivotMotorState = state;

        // set motor state
        switch (state) {
        case ON:
            // On
            this.pivotMotor.set(Constants.ArmPivot.PivotMotorSpeedForwards);
            SmartDashboard.putString("PivotMotorState", "On");

            break;

        case OFF:
            // Off
            this.pivotMotor.set(0);
            SmartDashboard.putString("PivotMotorState", "Off");

            break;

        case REVERSED:
            // Reversed
            this.pivotMotor.set(Constants.ArmPivot.PivotMotorSpeedBackwards);
            SmartDashboard.putString("PivotMotorState", "Reversed");

            break;

        default:
            this.setPivotMotorState(PivotMotorState.OFF);
        }
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
        return this.pivotEncoder.get();
    }

    public double getPivotEncoderFrequency() {
        return this.pivotEncoder.getFrequency();
    }

    public void periodic() {

        SmartDashboard.putNumber("PivotRot", getPivotMotorPosition());

    }
}