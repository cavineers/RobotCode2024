package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import com.revrobotics.CANSparkFlex;

public class Shooter extends SubsystemBase {

    public enum ShooterMotorState {
        ON,
        AMP, 
        OFF, 
        REVERSE
    }

    public CANSparkFlex shooterMotor = new CANSparkFlex(Constants.CanIDs.ShooterCanID, MotorType.kBrushless);

    // public DigitalImput m_intake (IR/April Tag stuff (maybe) TBD)

    public ShooterMotorState shooterMotorState = ShooterMotorState.OFF;
  


    public Shooter() {

        this.shooterMotor.setIdleMode(IdleMode.kCoast);

        this.shooterMotor.setSmartCurrentLimit(80); // TBD
        this.shooterMotor.setInverted(true);

        this.shooterMotor.getEncoder().setMeasurementPeriod(8);



    }

    public void setShooterMotorState(ShooterMotorState state) {

        this.shooterMotorState = state;

        switch (state) {

        case ON:
            this.shooterMotor.set(Constants.Shooter.ShooterForwardSpeed);
            // SmartDashboard.putString("Shooter", "Spinning");
            break;
        
        case AMP:
            this.shooterMotor.set(Constants.Shooter.AmpForwardSpeed);
            break;

        case REVERSE:
            this.shooterMotor.set(-.1);
            break;

        case OFF:
            this.shooterMotor.set(0.0);
            break;

        default:
            this.setShooterMotorState(ShooterMotorState.OFF);
        }
    }


    public ShooterMotorState getShooterMotorState() {
        return this.shooterMotorState;
    }

    public double getShooterMotorRPM() {
        return shooterMotor.getEncoder().getVelocity();
    }

    public double getShooterMotorSpeed() {
        return this.shooterMotor.get();
    }

    public void periodic() {
    }

}