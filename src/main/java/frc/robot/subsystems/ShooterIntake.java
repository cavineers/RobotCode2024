package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class ShooterIntake extends SubsystemBase {

    public enum ShooterMotorState {
        ON, 
        OFF, 
        REVERSE
    }

    public enum IntakeMotorState {
        ON, 
        OFF, 
        REVERSE, 
        RETRACT
    }

    public CANSparkMax shooterMotor = new CANSparkMax(Constants.CanIDs.ShooterCanID, MotorType.kBrushless);
    public CANSparkMax upperIntakeMotor = new CANSparkMax(Constants.CanIDs.UpperIntakeCanID, MotorType.kBrushless);
    public CANSparkMax lowerIntakeMotor = new CANSparkMax(Constants.CanIDs.LowerIntakeCanID, MotorType.kBrushless);

    public DigitalInput noteSensor = new DigitalInput(Constants.DIO.NoteSensor);

    // public DigitalImput m_intake (IR/April Tag stuff (maybe) TBD)

    public ShooterMotorState shooterMotorState = ShooterMotorState.OFF;
    public IntakeMotorState intakeMotorState = IntakeMotorState.OFF;

    public ShooterIntake() {

        this.shooterMotor.setIdleMode(IdleMode.kCoast);
        this.upperIntakeMotor.setIdleMode(IdleMode.kCoast);
        this.lowerIntakeMotor.setIdleMode(IdleMode.kCoast);

        this.shooterMotor.setSmartCurrentLimit(120); // TBD
        this.upperIntakeMotor.setSmartCurrentLimit(80); // TBD
        this.lowerIntakeMotor.setSmartCurrentLimit(80); // TBD

        this.shooterMotor.setInverted(true);
        this.upperIntakeMotor.setInverted(false);
        this.lowerIntakeMotor.setInverted(true);
        
    }

    public void setShooterMotorState(ShooterMotorState state) {

        this.shooterMotorState = state;

        switch (state) {

        case ON:
            this.shooterMotor.set(Constants.ShooterIntake.ShooterForwardSpeed);
            break;

        case REVERSE:
            this.shooterMotor.set(Constants.ShooterIntake.ShooterReverseSpeed);
            break;

        case OFF:
            this.shooterMotor.set(0.0);
            break;

        default:
            this.setShooterMotorState(ShooterMotorState.OFF);
        }
    }

    public void setIntakeMotorState(IntakeMotorState state) {

        this.intakeMotorState = state;

        switch (state) {

        case ON:
            this.upperIntakeMotor.set(Constants.ShooterIntake.UpperIntakeForwardSpeed);
            this.lowerIntakeMotor.set(Constants.ShooterIntake.LowerIntakeForwardSpeed);
            break;

        case REVERSE:
            this.upperIntakeMotor.set(Constants.ShooterIntake.UpperIntakeReverseSpeed);
            this.lowerIntakeMotor.set(Constants.ShooterIntake.LowerIntakeReverseSpeed);
            break;

        case RETRACT:
            this.upperIntakeMotor.set(Constants.ShooterIntake.UpperIntakeRetractSpeed);
            this.lowerIntakeMotor.set(Constants.ShooterIntake.LowerIntakeRetractSpeed);
            break;

        case OFF:
            this.upperIntakeMotor.set(0.0);
            this.lowerIntakeMotor.set(0.0);
            break;

        default:
            this.setIntakeMotorState(IntakeMotorState.OFF);

        }
    }

    public ShooterMotorState getShooterMotorState() {
        return this.shooterMotorState;
    }

    public IntakeMotorState getIntakeMotorState() {
        return this.intakeMotorState;
    }

    public double getShooterMotorRPM() {
        return shooterMotor.getEncoder().getVelocity();
    }

    public double getShooterMotorSpeed() {
        return this.shooterMotor.get();
    }

    public double getUpperIntakeMotorSpeed() {
        return this.upperIntakeMotor.get();
    }

    public double getLowerIntakeMotorSpeed() {
        return this.lowerIntakeMotor.get();
    }

    public void periodic() {

    }

}