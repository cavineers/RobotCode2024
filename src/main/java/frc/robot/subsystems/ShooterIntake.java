package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;

public class ShooterIntake extends SubsystemBase {

    
    public enum IntakeMotorState {
        ON,
        OFF,
        REVERSE
    }
    
    public CANSparkMax intakeMotor = new CANSparkMax(Constants.ShooterIntake.intakeCanID, MotorType.kBrushless);
    //public DigitalImput m_intake (IR/April Tag stuff (maybe) TBD)
    public IntakeMotorState intakeMotorState = IntakeMotorState.OFF;

    public ShooterIntake() {
        this.intakeMotor.setIdleMode(IdleMode.kBrake);
        this.intakeMotor.setSmartCurrentLimit(40);
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
        }
    } 

    public IntakeMotorState getIntakeMotorState() {
        return intakeMotorState;
    }

    public double getIntakeMotorSpeed() {
        return this.intakeMotor.get();
    }

    public void periodic() {

    }
}