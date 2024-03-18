package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.Timer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeNote extends Command {

    private Intake intake;
    private Shooter shooter;
    private boolean isDone;

    public IntakeNote(Intake intake, Shooter shooter) {
        this.intake = intake;
        this.shooter = shooter;
        this.addRequirements(intake);
        //this.addRequirements(shooter);
    }

    @Override
    public void initialize() {
        this.isDone = false;
    }

    @Override
    public void execute() {
        // SmartDashboard.putString("Intake", "Intaking");

        if (intake.getNoteSensor() == false) {
            intake.setIntakeMotorState(intake.intakeMotorState.ON);
            //shooter.setShooterMotorState(shooter.shooterMotorState.REVERSE);
        } else {
            intake.setIntakeMotorState(intake.intakeMotorState.OFF);
            this.isDone = true;
            //shooter.setShooterMotorState(shooter.shooterMotorState.OFF);
        }

    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakeMotorState(intake.intakeMotorState.OFF);
        //shooter.setShooterMotorState(shooter.shooterMotorState.OFF);
    }

    @Override
    public boolean isFinished() {
        return isDone;
    }

}
