package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    public enum ShooterMotorState {
        ON,
        AMP, 
        OFF, 
        REVERSE
    }

    public CANSparkMax shooterMotor = new CANSparkMax(Constants.CanIDs.ShooterCanID, MotorType.kBrushless);

    public SparkPIDController shooterPID = shooterMotor.getPIDController();

    // public DigitalImput m_intake (IR/April Tag stuff (maybe) TBD)

    public ShooterMotorState shooterMotorState = ShooterMotorState.OFF;

    public Shooter() {

        this.shooterMotor.setIdleMode(IdleMode.kCoast);

        this.shooterMotor.setSmartCurrentLimit(80); // TBD

        this.shooterMotor.setInverted(true);

        this.shooterMotor.getEncoder().setMeasurementPeriod(8);

        this.shooterPID.setIZone(0.0);
        this.shooterPID.setOutputRange(-1.0, 1.0);
        this.shooterPID.setP(Constants.Shooter.kP);
        this.shooterPID.setI(Constants.Shooter.kI);
        this.shooterPID.setD(Constants.Shooter.kD);
        this.shooterPID.setFF(Constants.Shooter.kF);

    }

    public void setShooterMotorState(ShooterMotorState state) {

        this.shooterMotorState = state;

        switch (state) {

        case ON:
            this.shooterMotor.set(Constants.Shooter.ShooterForwardSpeed);
            break;
        
        case AMP:
            this.shooterMotor.set(Constants.Shooter.AmpForwardSpeed);
            break;

        case REVERSE:
            this.shooterMotor.set(Constants.Shooter.ShooterReverseSpeed);
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
        SmartDashboard.putNumber("Shooter RPM", getShooterMotorRPM());
        SmartDashboard.putNumber("armPivotTriangleAngleFromPivotDegrees", Constants.ArmPivot.armPivotTriangleAngleFromPivotDegrees);
    }

}