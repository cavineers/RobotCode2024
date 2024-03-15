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

    public ShootRevWheels(Shooter shooter) {
        this.shooter = shooter;
        this.addRequirements(shooter);

    }

    @Override
    public void initialize() {
        this.isDone = false;
    }

    @Override
    public void execute() {
        shooter.setShooterMotorState(shooter.shooterMotorState.ON);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterMotorState(shooter.shooterMotorState.OFF);
    }

    @Override
    public boolean isFinished(){
        return this.isDone;
    }

}
