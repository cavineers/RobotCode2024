package frc.robot.commands.Shooter;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Amp extends Command {

    private boolean isDone;
    private Shooter shooter;
    private Intake intake;
    private Timer timer;
    private Timer timer2;

    private Blinkin blinkIn;

    public Amp(Shooter shooter, Intake intake, Blinkin blinkIn) {
        this.shooter = shooter;
        this.intake = intake;
        this.addRequirements(shooter);
        timer = new Timer();
        timer2 = new Timer();

        this.blinkIn = blinkIn;
    }

    // Set Motor State to ON / OFF
    @Override
    public void initialize() {
        this.isDone = false;

        timer.reset();
        timer2.reset();
        
        timer.start();

        blinkIn.lightsFire();

    }
    

    @Override
    public void execute() {
        // SmartDashboard.putString("Amp", "Amping");

        shooter.setShooterMotorState(shooter.shooterMotorState.AMP);
        if (timer.get()>.6) {
            intake.setIntakeMotorState(intake.intakeMotorState.ON);
        }

        if (intake.getNoteSensor()== false || timer.get()>3) {
           timer2.start();
           if (timer2.get()>0.5){
                this.isDone = true;
           }
        }

    }

    @Override
    public void end(boolean interrupted) {
        // SmartDashboard.putString("Amp", "Finished");

        shooter.setShooterMotorState(shooter.shooterMotorState.OFF);
        intake.setIntakeMotorState(intake.intakeMotorState.OFF);

        blinkIn.lightsDefault();
    }

    @Override
    public boolean isFinished() {
        return this.isDone;
    }

}
