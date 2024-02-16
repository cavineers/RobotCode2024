package frc.robot.commands.ShooterIntake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterIntake;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shoot extends Command {

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

        SmartDashboard.putString("Shooter", "Shooting");
        
        shooterIntake.setShooterMotorState(shooterIntake.shooterMotorState.ON);
        Timer.delay(.5);
        shooterIntake.setIntakeMotorState(shooterIntake.intakeMotorState.REVERSE);
        // shooterIntake.setIntakeMotorState(shooterIntake.intakeMotorState.OFF);
        Timer.delay(.5);
        // shooterIntake.setIntakeMotorState(shooterIntake.intakeMotorState.ON);
        Timer.delay(2);
        shooterIntake.setIntakeMotorState(shooterIntake.intakeMotorState.OFF);
        shooterIntake.setShooterMotorState(shooterIntake.shooterMotorState.OFF);
    }

    @Override
    public void end(boolean interrupted) {
        shooterIntake.setIntakeMotorState(shooterIntake.intakeMotorState.OFF);
        shooterIntake.setShooterMotorState(shooterIntake.shooterMotorState.OFF);
    }

    // @Override
    // public boolean isFinished() {
    // if (Timer.getFPGATimestamp() - this.m_timestamp >= 0 &&
    // Robot.m_robotContainer.driverJoystick.getRawButton(0)) {
    // this.isDone = true;
    // }
    // return this.isDone;
    // }

}
