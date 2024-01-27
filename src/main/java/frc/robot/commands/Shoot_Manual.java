package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ShooterIntake;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shoot_Manual extends Command{
    
    private boolean isDone = false;
    private double m_timestamp = Timer.getFPGATimestamp();
    private ShooterIntake shooterIntake;
    private Supplier<Double> triggerValue;

    public Shoot_Manual(ShooterIntake shooterIntake, Supplier<Double> triggerFunction) {
        this.shooterIntake = shooterIntake;
        this.triggerValue = triggerFunction;
        this.addRequirements(shooterIntake);
    }

    // Set Motor State to ON / OFF
    @Override
    public void initialize() {
        this.isDone = false;
    }

    @Override
    public void execute() {

        SmartDashboard.putString("Shooter", "Command Running Manual");

        double shooterMotorSpeed = triggerValue.get();

        shooterMotorSpeed = shooterMotorSpeed * Constants.ShooterIntake.shooterForwardSpeed;
        shooterIntake.setShooterMotorState(shooterIntake.shooterMotorState.ON);

        shooterIntake.shooterMotor.set(shooterMotorSpeed);
        
    }

    @Override
    public void end(boolean interrupted) {
        shooterIntake.setIntakeMotorState(shooterIntake.intakeMotorState.OFF);
        shooterIntake.setShooterMotorState(shooterIntake.shooterMotorState.OFF);
    }

    @Override
    public boolean isFinished() {
        if (Timer.getFPGATimestamp() - this.m_timestamp >= 0 && Robot.m_robotContainer.driverJoystick.getRawButton(0)) {
            this.isDone = true;
        }
        return this.isDone;
    }

    }
