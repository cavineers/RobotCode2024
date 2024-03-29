package frc.robot.commands.Shooter;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shoot_Manual extends Command {

    private boolean isDone = false;
    private double m_timestamp = Timer.getFPGATimestamp();
    private Shooter shooter;
    private Supplier<Double> triggerValue;

    public Shoot_Manual(Shooter shooter, Supplier<Double> triggerFunction) {
        this.shooter = shooter;
        this.triggerValue = triggerFunction;
        this.addRequirements(shooter);
    }

    // Set Motor State to ON / OFF
    @Override
    public void initialize() {
        this.isDone = false;
    }

    @Override
    public void execute() {

        // SmartDashboard.putString("Shooter", "Shooting Manual");

        double shooterMotorSpeed = triggerValue.get();

        // SmartDashboard.putNumber("RightTriggerValue", shooterMotorSpeed);
        // SmartDashboard.putNumber("ShooterMotorSpeed", shooter.getShooterMotorSpeed());

        shooterMotorSpeed = shooterMotorSpeed * Constants.Shooter.ShooterForwardSpeed;


        shooter.shooterMotor.set(shooterMotorSpeed);

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
