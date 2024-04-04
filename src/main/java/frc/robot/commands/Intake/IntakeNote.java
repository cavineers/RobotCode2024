package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.Timer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeNote extends Command {

    private Intake intake;
    private Shooter shooter;
    private boolean isDone;

    private int count;

    private Blinkin blinkIn;

    public IntakeNote(Intake intake, Shooter shooter, Blinkin blinkIn) {
        this.intake = intake;
        this.shooter = shooter;
        this.addRequirements(intake);
        //this.addRequirements(shooter);
        this.count = 0;
        this.blinkIn = blinkIn;
    }

    @Override
    public void initialize() {
        this.isDone = false;
        this.count = 0;
    }

    @Override
    public void execute() {
        // SmartDashboard.putString("Intake", "Intaking");

        if (intake.getNoteSensor() == false) {
            intake.setIntakeMotorState(intake.intakeMotorState.ON);
            this.count = 0;
            //shooter.setShooterMotorState(shooter.shooterMotorState.REVERSE);
        } else if (count >= 2) {
            intake.setIntakeMotorState(intake.intakeMotorState.OFF);
            this.isDone = true;
            //shooter.setShooterMotorState(shooter.shooterMotorState.OFF);
        }else{
            count++;
        }

    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakeMotorState(intake.intakeMotorState.OFF);
        
        if(intake.getNoteSensor()) {
            blinkIn.lightsOrange();
        }
    }

    @Override
    public boolean isFinished() {
        return isDone;
    }

}
