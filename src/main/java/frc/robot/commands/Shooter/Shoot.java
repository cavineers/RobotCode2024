package frc.robot.commands.Shooter;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Shoot extends Command {

    private double timestamp;
    private Shooter shooter;
    private Intake intake;
    private Timer timer;
    private boolean isDone;

    public Shoot(Shooter shooter, Intake intake) {
        this.shooter = shooter;
        this.intake = intake;
        this.addRequirements(shooter, intake);
        
        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        this.isDone = false;
    }

    @Override
    public void execute() {

        SmartDashboard.putString("Shooter", "Shooting");

        shooter.setShooterMotorState(shooter.shooterMotorState.ON);
        if (timer.get()>1) {
            intake.setIntakeMotorState(intake.intakeMotorState.ON);
        }

        if (intake.getNoteSensor()==false) {
           timer.reset();
           timer.start();
           if (timer.get()>0.5){
                this.isDone = true;
           }
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterMotorState(shooter.shooterMotorState.OFF);
        intake.setIntakeMotorState(intake.intakeMotorState.OFF);
        timer.stop();
    }

    @Override
    public boolean isFinished(){
        return this.isDone;
    }

}
