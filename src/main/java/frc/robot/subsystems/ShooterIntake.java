package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterIntake extends SubsystemBase {

    public enum ShooterMotorState{
        ON,
        OFF
    }

    public enum IntakeMotorState {
        ON,
        OFF,
        REVERSE
    }
    public enum FeederMotorState {
        ON,
        OFF,
        REVERSE
    }
    public CANSparkMax shooterMotor = new CANSparkMax(Constants.ShooterIntake.shooterCanID, MotorType.kBrushless);
    public CANSparkMax intakeMotor = new CANSparkMax(Constants.ShooterIntake.intakeCanID, MotorType.kBrushless);
    public CANSparkMax feederMotor = new CANSparkMax(Constants.ShooterIntake.feederCanID, MotorType.kBrushless);
    //public DigitalInput shooterMotor = new DigitalImput(Constants.DIO.shooterMotor);
    // public DigitalImput m_intake (IR/April Tag stuff (maybe) TBD)

    public ShooterMotorState shooterMotorState = ShooterMotorState.OFF;
    public IntakeMotorState intakeMotorState = IntakeMotorState.OFF;
    public FeederMotorState feederMotorState = FeederMotorState.OFF;
    public ShooterIntake() {
       
        this.shooterMotor.setIdleMode(IdleMode.kBrake);
        this.intakeMotor.setIdleMode(IdleMode.kBrake);
        this.feederMotor.setIdleMode(IdleMode.kBrake);

        this.shooterMotor.setSmartCurrentLimit(41); //TBD
        this.intakeMotor.setSmartCurrentLimit(41); //TBD
        this.feederMotor.setSmartCurrentLimit(41); //TBD
    }

    public void setShooterMotorState(ShooterMotorState state) {
        
        this.shooterMotorState = state;
        
        switch(state) {

            case ON:
            this.shooterMotor.set(40); //TBD
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
            this.intakeMotor.set(50); //TBD
            break;

            case REVERSE:
            this.intakeMotor.set(-50);
            break;

            case OFF:
            this.intakeMotor.set(0.0);
            break;
            
            default:
            this.setIntakeMotorState(IntakeMotorState.OFF);

        }
    } 

    public void setFeederMotorState(FeederMotorState state) {
        
        this.feederMotorState = state;
        
        switch(state) {

            case ON:
            this.feederMotor.set(15); //TBD
            break;

            case OFF:
            this.feederMotor.set(0.0);
            break;

            default:
            this.setFeederMotorState(FeederMotorState.OFF);
        }
    }

    public ShooterMotorState getShooterMotorState() {
        return this.shooterMotorState;
    }
    
    public IntakeMotorState getIntakeMotorState() {
        return intakeMotorState;
    }
    public FeederMotorState getFeederMotorState() {
        return this.feederMotorState;
    }
    public double getShooterMotorSpeed() {
        return shooterMotor.get();
    }
    
    public double getIntakeMotorSpeed() {
        return this.intakeMotor.get();
    }
    public double getFeederMotorSpeed() {
        return this.feederMotor.get();
    }
    public void periodic(){
    
    }
    
}