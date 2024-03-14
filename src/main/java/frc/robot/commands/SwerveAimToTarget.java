package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSubsystem;
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

    private final Supplier<Double> xSpdFunction, ySpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public SwerveAimToTarget(SwerveDriveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction,
            Supplier<Boolean> fieldOrientedFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;

        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
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

		// Create PID controller for turning to target
		PIDController turnController = new PIDController(1, 0, 0);
		turnController.setTolerance(0.1);
		
		//Check team to see if we need to be at 0 or 180
		if (DriverStation.getAlliance().isPresent() == true){
			Alliance team = DriverStation.getAlliance().get();
			if (team == Alliance.Blue){
				turnController.setSetpoint(180);
			} else {
				turnController.setSetpoint(0);
			}
		}else{
            return;
        }
		double turningSpeed = turnController.calculate(swerveSubsystem.getPose().getRotation().getDegrees());
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
        // swerveSubsystem.driveRelativeSpeeds(chassisSpeeds);


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