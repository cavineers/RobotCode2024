package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.lang.Math;
import java.util.function.Supplier;


import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveAimToTarget extends Command {

    private final SwerveDriveSubsystem swerveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    private final PIDController turnController = new PIDController(0.022, 0, 0.0);


    public SwerveAimToTarget(SwerveDriveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction,
            Supplier<Boolean> fieldOrientedFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;

        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

    	// Create PID controller for turning to target
		turnController.setTolerance(0.1);
		
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {

        // Get real-time joystick inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        

        // Apply deadband -- compensated for when the joystick value does not return to exactly zero
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;


        // Smooths driving for jerky joystick movement & eases acceleration
        if (fieldOrientedFunction.get()){
            xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMinSpeedMetersPerSecond;
            ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMinSpeedMetersPerSecond;
        } else {
            xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        }

	

		
		double turningSpeed = turnController.calculate(swerveSubsystem.getPose().getRotation().getDegrees(), visionSubsystem.getAngleToSpeaker());
		SmartDashboard.putNumber("TurningJoystick Input", turningSpeed);
        turningSpeed = turningLimiter.calculate(turningSpeed)
            * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        
        // Field relative Speeds
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());

        // Robot Relative Speeds
        // chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        
        // Convert chassis speeds to individual module states

        // UNCOMMENT TO ACTUALLY ALLOW DRIVINGS
        swerveSubsystem.driveRelativeSpeeds(chassisSpeeds);


    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
        swerveSubsystem.toggleIdleMode(IdleMode.kCoast);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}