package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterIntake;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shoot extends CommandBase{
    
    private boolean isDone = false;
    private double m_timestamp = Timer.getFPGATimestamp();
    private ShooterIntake shooterIntake;

    public Shoot(ShooterIntake shooterIntake) {
        this.shooterIntake = shooterIntake;
        this.addRequirements(shooterIntake);
    }

    // Set Motor State to ON / OFF
    @Override
    public void initialize() {
        this.isDone = false;
    }

    @Override
    public void execute() {

        SmartDashboard.putString("Shooter", "Command Running");
        
        shooterIntake.setShooterMotorState(shooterIntake.shooterMotorState.ON);
        Timer.delay(.5);
        shooterIntake.setFeederMotorState(shooterIntake.feederMotorState.ON);
        shooterIntake.setIntakeMotorState(shooterIntake.intakeMotorState.ON);
    }

    @Override
    public void end(boolean interrupted) {
        shooterIntake.setIntakeMotorState(ShooterIntake.IntakeMotorState.OFF);
        shooterIntake.setShooterMotorState(ShooterIntake.ShooterMotorState.OFF);
    }

    @Override
    public boolean isFinished() {
        if (Timer.getFPGATimestamp() - this.m_timestamp >= 0 && Robot.m_robotContainer.driverJoystick.getRawButton(0)) {
            this.isDone = true;
        }
        return this.isDone;
    }

    }
