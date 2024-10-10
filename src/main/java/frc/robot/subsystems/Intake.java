package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    public enum IntakeMotorState {
        ON, 
        OFF, 
        REVERSE, 
        RETRACT
    }

    public CANSparkMax upperIntakeMotor = new CANSparkMax(Constants.CanIDs.UpperIntakeCanID, MotorType.kBrushless);
    public CANSparkMax lowerIntakeMotor = new CANSparkMax(Constants.CanIDs.LowerIntakeCanID, MotorType.kBrushless);

    public DigitalInput noteSensorLeft = new DigitalInput(Constants.DIO.NoteSensorLeft);
    public DigitalInput noteSensorRight = new DigitalInput(Constants.DIO.NoteSensorRight);

    public IntakeMotorState intakeMotorState = IntakeMotorState.OFF;

    private Blinkin blinkin;

    private boolean alreadySet;
    
    public Intake(Blinkin blinkin) {

        this.upperIntakeMotor.setIdleMode(IdleMode.kBrake);
        this.lowerIntakeMotor.setIdleMode(IdleMode.kBrake);

        this.upperIntakeMotor.setSmartCurrentLimit(35); // TBD
        this.lowerIntakeMotor.setSmartCurrentLimit(35); // TBD

        this.upperIntakeMotor.setInverted(false);
        this.lowerIntakeMotor.setInverted(true);
        this.blinkin = blinkin;
        this.alreadySet = false;
    }

    public void setIntakeMotorState(IntakeMotorState state) {

        this.intakeMotorState = state;

        switch (state) {

        case ON:
            this.upperIntakeMotor.set(Constants.Intake.UpperIntakeForwardSpeed);
            this.lowerIntakeMotor.set(Constants.Intake.LowerIntakeForwardSpeed);
            // SmartDashboard.putString("Intake", "Intaking");
            break;

        case REVERSE:
            this.upperIntakeMotor.set(Constants.Intake.UpperIntakeReverseSpeed);
            this.lowerIntakeMotor.set(Constants.Intake.LowerIntakeReverseSpeed);
            break;

        case RETRACT:
            this.upperIntakeMotor.set(Constants.Intake.UpperIntakeRetractSpeed);
            this.lowerIntakeMotor.set(Constants.Intake.LowerIntakeRetractSpeed);
            break;

        case OFF:
            this.upperIntakeMotor.set(0.0);
            this.lowerIntakeMotor.set(0.0);
            break;

        default:
            this.setIntakeMotorState(IntakeMotorState.OFF);
        }
    }

    public IntakeMotorState getIntakeMotorState() {
        return this.intakeMotorState;
    }

    public double getUpperIntakeMotorSpeed() {
        return this.upperIntakeMotor.get();
    }

    public double getLowerIntakeMotorSpeed() {
        return this.lowerIntakeMotor.get();
    }

    public boolean getNoteSensor() {
        return (!this.noteSensorLeft.get() || !this.noteSensorRight.get());
    }

}