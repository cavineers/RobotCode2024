package frc.robot.commands.Shooter;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ShootRevWheels extends Command {

    private double timestamp;
    private Shooter shooter;
    private boolean isDone;
    private boolean state;

    public ShootRevWheels(Shooter shooter, boolean state) {
        this.shooter = shooter;
        this.state = state;
        this.addRequirements(shooter);

    }

    @Override
    public void initialize() {
        if (state == true){
            shooter.setShooterMotorState(shooter.shooterMotorState.ON);
        }else{
            shooter.setShooterMotorState(shooter.shooterMotorState.OFF);
        }
        this.isDone = true;
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished(){
        return this.isDone;
    }

}
