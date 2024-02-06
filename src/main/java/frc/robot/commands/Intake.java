package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterIntake;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends Command{
    
    private boolean isDone = false;
    private double m_timestamp;
    private ShooterIntake shooterIntake;

    public Intake(ShooterIntake shooterIntake) {
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

        SmartDashboard.putString("Intake", "Intaking");

        if (shooterIntake.noteSensor.get() == false) {
            shooterIntake.setIntakeMotorState(shooterIntake.intakeMotorState.ON);
        } else {
            shooterIntake.setIntakeMotorState(shooterIntake.intakeMotorState.OFF);
        }

    }

    @Override
    public void end(boolean interrupted) {
        shooterIntake.setIntakeMotorState(shooterIntake.intakeMotorState.OFF);
    }

    // @Override
    // public boolean isFinished() {
    //     if (Timer.getFPGATimestamp() - this.m_timestamp >= 0 && Robot.m_robotContainer.driverJoystick.getRawButton(0)) {
    //         this.isDone = true;
    //     }
    //     return this.isDone;
    // }

}
