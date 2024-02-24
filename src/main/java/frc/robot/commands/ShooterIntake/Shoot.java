package frc.robot.commands.ShooterIntake;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterIntake;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shoot extends Command {

    private boolean isDone = false;
    private double m_timestamp = Timer.getFPGATimestamp();
    private ShooterIntake shooterIntake;

    private double distanceMeters;
    private double shooterDistanceFromGround = 0;
    private double shootingHeight = (2.0574 - shooterDistanceFromGround);
    private double requiredShooterAngle;
    private double requiredShooterVelocity;
    private double requiredShooterRPM;

    ShuffleboardTab tab = Shuffleboard.getTab("Robot");

    private GenericEntry distanceEntry = tab
        .add("Distance to target (Meters)", 0)
        .getEntry();

    public Shoot(ShooterIntake shooterIntake) {
        this.shooterIntake = shooterIntake;
        this.addRequirements(shooterIntake);
    }

    // Set Motor State to ON / OFF
    @Override
    public void initialize() {
        this.isDone = false;
    }

    @Override
    public void execute() {

        distanceMeters = distanceEntry.getDouble(0);

        requiredShooterAngle = (Math.atan(1/(distanceMeters/(2*shootingHeight))));
        requiredShooterVelocity = ((Math.sqrt(2*9.81*shootingHeight))/(Math.sin(requiredShooterAngle)));
        requiredShooterRPM = ((60*requiredShooterVelocity)/(.102*Math.PI));

        SmartDashboard.putString("Shooter", "Shooting");
        SmartDashboard.putNumber("Shooter RPM", shooterIntake.getShooterMotorRPM());
        SmartDashboard.putNumber("Required Shooter Angle", requiredShooterAngle);
        SmartDashboard.putNumber("Required Shooter Velocity", requiredShooterVelocity);
        SmartDashboard.putNumber("Required Shooter RPM", requiredShooterRPM);

        shooterIntake.setShooterMotorState(shooterIntake.shooterMotorState.ON);
        Timer.delay(2.5);
        shooterIntake.setIntakeMotorState(shooterIntake.intakeMotorState.ON);
        // shooterIntake.setIntakeMotorState(shooterIntake.intakeMotorState.OFF);
        Timer.delay(.5);
        // shooterIntake.setIntakeMotorState(shooterIntake.intakeMotorState.ON);
        Timer.delay(3);
        shooterIntake.setIntakeMotorState(shooterIntake.intakeMotorState.OFF);
        shooterIntake.setShooterMotorState(shooterIntake.shooterMotorState.OFF);
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
