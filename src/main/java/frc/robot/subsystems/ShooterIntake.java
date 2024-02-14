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
    public CANSparkMax intakeMotor = new CANSparkMax(Constants.CanIDs.IntakeCanID, MotorType.kBrushless);
    public CANSparkMax intake2ndMotor = new CANSparkMax(Constants.CanIDs.Intake2ndCanID, MotorType.kBrushless);

    public DigitalInput noteSensor = new DigitalInput(Constants.DIO.NoteSensor);

    // public DigitalImput m_intake (IR/April Tag stuff (maybe) TBD)

    public ShooterMotorState shooterMotorState = ShooterMotorState.OFF;
    public IntakeMotorState intakeMotorState = IntakeMotorState.OFF;

    public ShooterIntake() {

        this.shooterMotor.setIdleMode(IdleMode.kBrake);
        this.intakeMotor.setIdleMode(IdleMode.kBrake);
        this.intakeMotor.setIdleMode(IdleMode.kBrake);

        this.shooterMotor.setSmartCurrentLimit(41); // TBD
        this.intakeMotor.setSmartCurrentLimit(41); // TBD
        this.intake2ndMotor.setSmartCurrentLimit(41); // TBD

        this.intake2ndMotor.setInverted(true);
        this.intake2ndMotor.follow(intakeMotor);
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
            this.intakeMotor.set(Constants.ShooterIntake.IntakeForwardSpeed);
            break;

        case REVERSE:
            this.intakeMotor.set(Constants.ShooterIntake.IntakeReverseSpeed);
            break;

        case RETRACT:
            this.intakeMotor.set(Constants.ShooterIntake.IntakeRetractSpeed);
            break;

        case OFF:
            this.intakeMotor.set(0.0);
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

    public double getShooterMotorSpeed() {
        return this.shooterMotor.get();
    }

    public double getIntakeMotorSpeed() {
        return this.intakeMotor.get();
    }

    public void periodic() {

    }

}