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
    private Timer timer2;
    private boolean isDone;

    private ShuffleboardTab tab = Shuffleboard.getTab("Shooter Params");
    private GenericEntry shooterWaitGetter;

    public Shoot(Shooter shooter, Intake intake) {
        this.shooter = shooter;
        this.intake = intake;
        this.addRequirements(shooter, intake);
        
        timer = new Timer();
        timer2 = new Timer();
        this.shooterWaitGetter = tab.add("Shooter Wait Time", 2).getEntry();
    }

    @Override
    public void initialize() {
        timer.reset();
        timer2.reset();
        timer.start();

        this.isDone = false;
    }

    @Override
    public void execute() {

        // SmartDashboard.putString("Shooter", "Shooting");

        shooter.setShooterMotorState(shooter.shooterMotorState.ON);
        if (timer.get()>.75) {
            intake.setIntakeMotorState(intake.intakeMotorState.ON);
        }

        if (intake.getNoteSensor()== false || timer.get()>3) {
           timer2.start();
           if (timer2.get()>0.5){
                this.isDone = true;
           }
        }

        // SmartDashboard.putNumber("Timer1", timer.get());
        // SmartDashboard.putNumber("Timer2", timer2.get());
        
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterMotorState(shooter.shooterMotorState.OFF);
        intake.setIntakeMotorState(intake.intakeMotorState.OFF);
        timer.stop();
        // SmartDashboard.putString("Shooter", "Done Shooting");
    }

    @Override
    public boolean isFinished(){
        return this.isDone;
    }

}
