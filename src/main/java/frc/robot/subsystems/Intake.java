package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;


public class Intake extends SubsystemBase {

    public enum IntakeMotorState {
        ON, 
        OFF, 
        REVERSE, 
        RETRACT
    }

    public CANSparkMax upperIntakeMotor = new CANSparkMax(Constants.CanIDs.UpperIntakeCanID, MotorType.kBrushless);
    public CANSparkMax lowerIntakeMotor = new CANSparkMax(Constants.CanIDs.LowerIntakeCanID, MotorType.kBrushless);

    public DigitalInput noteSensor = new DigitalInput(Constants.DIO.NoteSensor);

    public IntakeMotorState intakeMotorState = IntakeMotorState.OFF;

    private Blinkin blinkin;
    
    public Intake() {

        this.upperIntakeMotor.setIdleMode(IdleMode.kCoast);
        this.lowerIntakeMotor.setIdleMode(IdleMode.kCoast);

        this.upperIntakeMotor.setSmartCurrentLimit(80); // TBD
        this.lowerIntakeMotor.setSmartCurrentLimit(80); // TBD

        this.upperIntakeMotor.setInverted(false);
        this.lowerIntakeMotor.setInverted(true);
        this.blinkin = RobotContainer.blinkin;
    }

    public void setIntakeMotorState(IntakeMotorState state) {

        this.intakeMotorState = state;

        switch (state) {

        case ON:
            this.upperIntakeMotor.set(Constants.Intake.UpperIntakeForwardSpeed);
            this.lowerIntakeMotor.set(Constants.Intake.LowerIntakeForwardSpeed);
            SmartDashboard.putString("Intake", "Intaking");
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
        return !this.noteSensor.get();
    }

    public void periodic() {
        SmartDashboard.putBoolean("INTAKE IR", getNoteSensor());

        if(getNoteSensor() == true) {
            this.blinkin.lightsOrange();
        }
    }

}