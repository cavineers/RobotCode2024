package frc.robot.commands.Shooter;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Amp extends Command {

    private boolean isDone = false;
    private double m_timestamp = Timer.getFPGATimestamp();
    private Shooter shooter;

    public Amp(Shooter shooter) {
        this.shooter = shooter;
        this.addRequirements(shooter);
    }

    // Set Motor State to ON / OFF
    @Override
    public void initialize() {
        this.isDone = false;
    }

    @Override
    public void execute() {

        SmartDashboard.putString("Amp", "Amping");

        shooter.setShooterMotorState(shooter.shooterMotorState.AMP);

    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterMotorState(shooter.shooterMotorState.OFF);
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
