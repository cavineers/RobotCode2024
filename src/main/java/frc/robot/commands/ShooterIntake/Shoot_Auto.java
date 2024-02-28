package frc.robot.commands.ShooterIntake;

import com.revrobotics.CANSparkBase;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmBase;
import frc.robot.subsystems.ArmPivot;
import frc.robot.subsystems.ShooterIntake;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shoot_Auto extends Command {

    private boolean isDone = false;
    private double m_timestamp = Timer.getFPGATimestamp();
    private ShooterIntake shooterIntake;
    private ArmBase armBase;
    private ArmPivot armPivot;

    private double shooterDistanceFromGroundMeters;
    private double shootingHeight;
    private double requiredShooterAngle;
    private double requiredShooterVelocity;
    private double requiredShooterRPM;
    private double requiredArmPivotAngleDegrees;
    private double currentShooterAngleFromBaseline;

    private double distanceMeters;

    ShuffleboardTab tab = Shuffleboard.getTab("Robot");

    private GenericEntry distanceEntry = tab
        .add("Distance to target (Meters)", 0)
        .getEntry();

    public Shoot_Auto(ShooterIntake shooterIntake, ArmPivot armPivot, ArmBase armBase) {
        this.shooterIntake = shooterIntake;
        this.armPivot = armPivot;
        this.armBase = armBase;
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

        shootingHeight = (Constants.ShooterIntake.shootingVertexHeightMeters - calculateCurrentShooterHeight());
        SmartDashboard.putNumber("Shooting Height", shootingHeight);

        distanceMeters = distanceEntry.getDouble(0);

        SmartDashboard.putString("Shooter", "Shooting");

        // Shooting Sequence
        armPivot.setArmPivotAngle(calculateRequiredArmPivotAngle(calculateRequiredAngle(distanceMeters)));
        setShooterPIDReference(distanceMeters);
        Timer.delay(1);
        shooterIntake.setIntakeMotorState(shooterIntake.intakeMotorState.ON);
        Timer.delay(.5);
        Timer.delay(1.5);
        shooterIntake.setIntakeMotorState(shooterIntake.intakeMotorState.OFF);
        shooterIntake.setShooterMotorState(shooterIntake.shooterMotorState.OFF);

        SmartDashboard.putString("Shooter", "Finished");
        
    }

    @Override
    public void end(boolean interrupted) {
        shooterIntake.setIntakeMotorState(shooterIntake.intakeMotorState.OFF);
        shooterIntake.setShooterMotorState(shooterIntake.shooterMotorState.OFF);
    }

    
    public double calculateRequiredAngle(Double distance) {

        requiredShooterAngle = (Math.atan(1/(distance/(2*shootingHeight))));

        SmartDashboard.putNumber("Required Shooter Angle", requiredShooterAngle);

        return requiredShooterAngle;
    }

    public double calculateRequiredVelocity(Double angle) {

        requiredShooterVelocity = ((Math.sqrt(2*9.81*shootingHeight))/(Math.sin(angle)));
        requiredShooterRPM = ((60*requiredShooterVelocity)/(.102*Math.PI));

        SmartDashboard.putNumber("Required Shooter Velocity", requiredShooterVelocity);
        SmartDashboard.putNumber("Required Shooter RPM", requiredShooterRPM);

        return requiredShooterRPM;
    }

    public double calculateCurrentShooterHeight() {

        shooterDistanceFromGroundMeters = armBase.getGantryHeightMeters() + (Constants.ArmPivot.armPivotDistanceFromShooterMeters * Math.sin(Math.toRadians(armPivot.getArmPivotHypToBaseline())));

        return shooterDistanceFromGroundMeters;
    }

    public double calculateCurrentShooterAngle() {

        currentShooterAngleFromBaseline = 180 - (Constants.ArmPivot.armPivotJointAngleDegrees + armPivot.getArmPivotAngle());

        return currentShooterAngleFromBaseline;
    }

    public double calculateRequiredArmPivotAngle(Double requiredShooterAngleDegrees) {
         
        requiredArmPivotAngleDegrees = 180 - (Constants.ArmPivot.armPivotJointAngleDegrees + armPivot.getArmPivotAngle());

        return requiredArmPivotAngleDegrees;
    }

    public void setShooterPIDReference(Double distanceFromSpeaker) {
        shooterIntake.shooterPID.setReference(calculateRequiredVelocity(calculateRequiredAngle(distanceFromSpeaker)), CANSparkBase.ControlType.kVelocity);
        SmartDashboard.putNumber("ShootPID Val", calculateRequiredVelocity(calculateRequiredAngle(distanceFromSpeaker)));
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
