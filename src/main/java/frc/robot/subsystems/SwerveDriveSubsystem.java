package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder.BackendKind;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.RobotContainer;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;



public class SwerveDriveSubsystem extends SubsystemBase {

    public AutoBuilder autoBuilder;

    public VisionSubsystem visionSubsystem;

    Optional<Alliance> ally = DriverStation.getAlliance();
    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveCanID, 
        DriveConstants.kFrontLeftTurningCanID, 
        DriveConstants.kFrontLeftDriveEncoderReversed, 
        DriveConstants.kFrontLeftTurningEncoderReversed,
        DriveConstants.kFrontLeftAbsoluteEncoderPort, 
        DriveConstants.kFrontLeftAbsoluteEncoderOffset);
    
    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveCanID, 
        DriveConstants.kFrontRightTurningCanID, 
        DriveConstants.kFrontRightDriveEncoderReversed, 
        DriveConstants.kFrontRightTurningEncoderReversed,
        DriveConstants.kFrontRightAbsoluteEncoderPort, 
        DriveConstants.kFrontRightAbsoluteEncoderOffset);

    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveCanID, 
        DriveConstants.kBackLeftTurningCanID, 
        DriveConstants.kBackLeftDriveEncoderReversed, 
        DriveConstants.kBackLeftTurningEncoderReversed,
        DriveConstants.kBackLeftAbsoluteEncoderPort, 
        DriveConstants.kBackLeftAbsoluteEncoderOffset);

    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.kBackRightDriveCanID, 
        DriveConstants.kBackRightTurningCanID, 
        DriveConstants.kBackRightDriveEncoderReversed, 
        DriveConstants.kBackRightTurningEncoderReversed,
        DriveConstants.kBackRightAbsoluteEncoderPort, 
        DriveConstants.kBackRightAbsoluteEncoderOffset);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP); 

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        new Translation2d(DriveConstants.kTrackWidth / 2.0, DriveConstants.kWheelBase / 2.0),
        new Translation2d(DriveConstants.kTrackWidth / 2.0, -DriveConstants.kWheelBase / 2.0),
        new Translation2d(-DriveConstants.kTrackWidth / 2.0, DriveConstants.kWheelBase / 2.0),
        new Translation2d(-DriveConstants.kTrackWidth / 2.0, -DriveConstants.kWheelBase / 2.0)
    );

    private final Field2d m_field = new Field2d();


    public double getHeading(){
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
        m_kinematics, 
        getRotation2d(), 
        getPositions());

    public double getFLAbsolutePosition(){
        return frontLeft.getAbsolutePosition();
    }
    public double getFRAbsolutePosition(){
        return frontRight.getAbsolutePosition();
    }
    public double getBRAbsolutePosition(){
        return backRight.getAbsolutePosition();
    }
    public double getBLAbsolutePosition(){
        return backLeft.getAbsolutePosition();
    }

    private Boolean flipField(){
        if (ally.get() == DriverStation.Alliance.Red){
            return true;
        } else {
            return false;
        }
    }

    SwerveDriveOdometry m_odometer = m_odometry;

    // POSE ESTIMATOR
    public Pose2d getOdometerPose() {
            return m_odometer.getPoseMeters();
    }

   
    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
        m_kinematics,
        getRotation2d(),
        getPositions(),
        getOdometerPose());

    private Pose2d updatePoseWithVision(){

        Pose2d currentPose = poseEstimator.update(getRotation2d(), getPositions());
        SmartDashboard.putNumber("PoseX1", currentPose.getX());
        SmartDashboard.putNumber("PoseY1", currentPose.getY());
        Optional<EstimatedRobotPose> visionPose = visionSubsystem.getRobotPoseFieldRelative(currentPose);
        if (visionPose.isEmpty()) {
            SmartDashboard.putBoolean("Has Tags", false);
            return currentPose;
        }
        SmartDashboard.putBoolean("Has Tags", true);
        Pose2d visionPose2d = visionPose.get().estimatedPose.toPose2d();
        poseEstimator.addVisionMeasurement(visionPose2d, visionPose.get().timestampSeconds);
        SmartDashboard.putNumber("PoseX2", visionPose2d.getX());
        SmartDashboard.putNumber("PoseY2", visionPose2d.getY());
        return poseEstimator.getEstimatedPosition();
    }

    public SwerveDriveSubsystem(VisionSubsystem visionSubsystem) {
        //Delay reset of navx for proper initialization
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
        this.visionSubsystem = visionSubsystem;

        this.updatePoseWithVision();
        // Configure the AutoBuilder last
        AutoBuilder.configureHolonomic(
            this::updatePoseWithVision, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRelativeSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                4.5, // Max module speed, in m/s
                0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            this::flipField, // Whether to flip the path
            this // Reference to this subsystem to set requirements
        );
    }

    public void zeroHeading() {
        gyro.reset();
    }

    

    private ChassisSpeeds getChassisSpeeds(){
        return m_kinematics.toChassisSpeeds(
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        );
        // return ChassisSpeeds.fromFieldRelativeSpeeds(
        //     speeds.vxMetersPerSecond,
        //     speeds.vyMetersPerSecond,
        //     speeds.omegaRadiansPerSecond,
        //     getRotation2d()
        // );
    }

    public SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()};
    }
    public void resetOdometry(Pose2d pose) {
        m_odometer.resetPosition(getRotation2d(), getPositions(), pose);
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void resetEncoders(){
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }
 
    public void periodic(){
        m_odometer.update(getRotation2d(), getPositions());
        updatePoseWithVision();
        m_field.setRobotPose(this.updatePoseWithVision());
        SmartDashboard.putData("Field", m_field);
        SmartDashboard.putNumber("Front Right Rel", this.frontRight.getTurningPosition());
        SmartDashboard.putNumber("Front Right Abs", this.frontRight.getAbsolutePosition());
        SmartDashboard.putNumber("Back Right Rel", this.backRight.getTurningPosition());
        SmartDashboard.putNumber("Back Right Abs", this.backRight.getAbsolutePosition());
        SmartDashboard.putNumber("Heading", getHeading());
        SmartDashboard.putNumber("Roll", getRoll());
        SmartDashboard.putNumber("Pitch", getPitch());
    }
    
    private void driveRelativeSpeeds(ChassisSpeeds relativeSpeeds){
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(relativeSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void setEncoders() {
        frontLeft.setEncoder();
        frontRight.setEncoder();
        backLeft.setEncoder();
        backRight.setEncoder();
    }

    public void toggleIdleMode(IdleMode mode) {
        frontLeft.toggleIdleMode(mode);
        frontRight.toggleIdleMode(mode);
        backLeft.toggleIdleMode(mode);
        backRight.toggleIdleMode(mode);

    }

    public double getRoll(){
        return gyro.getRoll();
    }

    public double getPitch(){
        return gyro.getPitch();
    }
}