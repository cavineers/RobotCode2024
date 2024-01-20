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
        OFF,
    }

    public CANSparkMax shooterMotor = new CANSparkMax(Constants.ShooterIntake.shooterCanID, MotorType.kBrushless);
    //public DigitalInput shooterMotor = new DigitalImput(Constants.DIO.shooterMotor);
    public ShooterMotorState shooterMotorState = ShooterMotorState.OFF;
   
    public ShooterIntake() {
       
        this.shooterMotor.setIdleMode(IdleMode.kBrake);
        this.shooterMotor.setSmartCurrentLimit(30); //TBD
    }

    public void setShooterMotorState(ShooterMotorState state) {
        
        this.shooterMotorState = state;
        
        switch(state) {

            case ON:
            this.shooterMotor.set(40); //TBD
            break;

            case OFF:
            this.shooterMotor.set(0);
            break;

            default:
            this.setShooterMotorState(ShooterMotorState.OFF);
        }
    }

    public double getShooterMotorSpeed() {
        return shooterMotor.get();
    }

    public ShooterMotorState getShooterMotorState() {
        return this.shooterMotorState;
    }

    public void periodic(){
    
    }
}