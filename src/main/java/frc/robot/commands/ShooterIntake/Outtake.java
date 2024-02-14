package frc.robot.commands.ShooterIntake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterIntake;

public class Outtake extends Command {

    private boolean isDone = false;
    private double m_timestamp;
    private ShooterIntake shooterIntake;

    public Outtake(ShooterIntake shooterIntake) {
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

        SmartDashboard.putString("Intake", "Outtaking");

        shooterIntake.setIntakeMotorState(shooterIntake.intakeMotorState.REVERSE);
    }

    @Override
    public void end(boolean interrupted) {
        shooterIntake.setIntakeMotorState(shooterIntake.intakeMotorState.OFF);
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
