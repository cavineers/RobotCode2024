package frc.robot.commands.ShooterIntake;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmPivot;
import frc.robot.subsystems.ShooterIntake;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shoot_Auto extends Command {

    private boolean isDone = false;
    private double m_timestamp = Timer.getFPGATimestamp();
    private ShooterIntake shooterIntake;
    private ArmPivot armPivot;

    private double distanceMeters;

    ShuffleboardTab tab = Shuffleboard.getTab("Robot");

    private GenericEntry distanceEntry = tab
        .add("Distance to target (Meters)", 0)
        .getEntry();

    public Shoot_Auto(ShooterIntake shooterIntake, ArmPivot armPivot) {
        this.shooterIntake = shooterIntake;
        this.armPivot = armPivot;
        this.addRequirements(shooterIntake);
        this.addRequirements(armPivot);
    }

    // Set Motor State to ON / OFF
    @Override
    public void initialize() {
        this.isDone = false;
    }

    @Override
    public void execute() {

        distanceMeters = distanceEntry.getDouble(0);

        SmartDashboard.putString("Shooter", "Shooting");

        // Shooting Sequence
        shooterIntake.setShooterPIDReference(distanceMeters);
        Timer.delay(2.5);
        shooterIntake.setIntakeMotorState(shooterIntake.intakeMotorState.ON);
        Timer.delay(.5);
        Timer.delay(3);
        shooterIntake.setIntakeMotorState(shooterIntake.intakeMotorState.OFF);
        shooterIntake.setShooterMotorState(shooterIntake.shooterMotorState.OFF);

        SmartDashboard.putString("Shooter", "Finished");
    }

    @Override
    public void end(boolean interrupted) {
        shooterIntake.setIntakeMotorState(shooterIntake.intakeMotorState.OFF);
        shooterIntake.setShooterMotorState(shooterIntake.shooterMotorState.OFF);
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
