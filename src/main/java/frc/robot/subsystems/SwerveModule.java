package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

import edu.wpi.first.math.MathUtil;

public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    private final CANcoder absoluteEncoder;
    private final double absoluteEncoderOffsetDeg;
    private final int id;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset) {
        
        this.id = absoluteEncoderId;
        this.absoluteEncoderOffsetDeg = absoluteEncoderOffset;
        this.absoluteEncoder = new CANcoder(absoluteEncoderId);

        // Configure the CANcoder for basic use
        CANcoderConfiguration configs = new CANcoderConfiguration();
        // This CANcoder should report absolute position from [-0.5, 0.5) rotations,
        configs.MagnetSensor.MagnetOffset = absoluteEncoderOffset;
        configs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        configs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        // Write these configs to the CANcoder
        absoluteEncoder.getConfigurator().apply(configs);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.setIdleMode(IdleMode.kCoast);
        turningMotor.setIdleMode(IdleMode.kCoast);

        driveMotor.setSmartCurrentLimit(40);
        turningMotor.setSmartCurrentLimit(40);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);
        
        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getAbsolutePosition(){
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public double getEncoderPosition(){
        return turningEncoder.getPosition();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            getDrivePosition(), new Rotation2d(getTurningPosition()));
    }
    
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public void toggleIdleMode(IdleMode mode) {
        turningMotor.setIdleMode(mode);
        driveMotor.setIdleMode(mode);
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(0);
    }

    private double getDistanceToHome(){ //Rotations
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI; // Convert to radians
    }

    public void setEncoder() { // Takes radians
        double distance = this.getDistanceToHome();
        turningEncoder.setPosition(distance);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    }
    
    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}