package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberRight extends SubsystemBase {

    // Initialize the climber motor
    public CANSparkMax rightClimberMotor = new CANSparkMax(Constants.CanIDs.RightClimberCanID, MotorType.kBrushless);

    // Initialize the limit switch
    public DigitalInput rightClimberTopLimitSwitch = new DigitalInput(Constants.DIO.RightClimberTopLimitSwitch);
    public DigitalInput rightClimberBottomLimitSwitch = new DigitalInput(Constants.DIO.RightClimberBottomLimitSwitch);

    // Motor states
    public enum RightClimberMotorState {
        ON, 
        OFF, 
        REVERSED
    }

    // Initial Motor State
    public RightClimberMotorState rightClimberMotorState = RightClimberMotorState.OFF;

    public ClimberRight() {
        this.rightClimberMotor.setIdleMode(IdleMode.kBrake);

        this.rightClimberMotor.setInverted(false);

        // Set the amp limit when specified - TBD
        this.rightClimberMotor.setSmartCurrentLimit(51);
    }

    // Allow for changing motor states
    public void setRightClimberMotorState(RightClimberMotorState state) {

        // Set the current state
        this.rightClimberMotorState = state;

        switch (state) {
        case ON:
            // On: Set the extension speed of the climber
            this.rightClimberMotor.set(Constants.Climber.ClimberExtensionSpeed);
            break;
        case OFF:
            // Off: Set the speed to zero
            this.rightClimberMotor.set(0.0);
            break;
        case REVERSED:
            // Reversed: Set the reversal speed of the climber
            this.rightClimberMotor.set(Constants.Climber.ClimberExtensionSpeedRev);
            break;
        default:
            this.setRightClimberMotorState(RightClimberMotorState.OFF);
        }
    }

    // Getters and setters

    public boolean getLimitSwitch(String orientation) {
        boolean switched;

        if (orientation == "top") {
            switched = this.rightClimberTopLimitSwitch.get();
            System.out.println("right top");

        } else {
            switched = this.rightClimberBottomLimitSwitch.get();
            System.out.println("right bottom");
        }

        return switched;
    }

    // Set the motor's position (given in rotations)
    public void setRightClimberMotorPosition(double position) {
        this.rightClimberMotor.getEncoder().setPosition(position);
    }

    // Set the motor's speed (value between -1 and 1)
    public void setRightClimberMotorSpeed(double speed) {
        this.rightClimberMotor.set(speed);
    }

    // Get motor position (value returned in number of rotations)
    public double getRightClimberMotorPosition() {
        return this.rightClimberMotor.getEncoder().getPosition();
    }

    // Get motor speed (value between -1 and 1)
    public double getRightClimberMotorSpeed() {
        return this.rightClimberMotor.get();
    }

    // Get the current state of the motor
    public RightClimberMotorState getRightClimberMotorState() {
        return this.rightClimberMotorState;
    }

    public CANSparkMax getRightClimberMotor() {
        return this.rightClimberMotor;
    }

    @Override
    public void periodic() {
    }
}