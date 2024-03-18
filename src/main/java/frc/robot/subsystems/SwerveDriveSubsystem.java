package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
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
import frc.robot.Constants.CanIDs;
import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
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

    private double gyroZero;
    Optional<Alliance> ally = DriverStation.getAlliance();
    private final SwerveModule frontLeft = new SwerveModule(
        CanIDs.kFrontLeftDriveCanID, 
        CanIDs.kFrontLeftTurningCanID, 
        DriveConstants.kFrontLeftDriveEncoderReversed, 
        DriveConstants.kFrontLeftTurningEncoderReversed,
        CanIDs.kFrontLeftAbsoluteEncoderPort, 
        DriveConstants.kFrontLeftAbsoluteEncoderOffset);
    
    private final SwerveModule frontRight = new SwerveModule(
        CanIDs.kFrontRightDriveCanID, 
        CanIDs.kFrontRightTurningCanID, 
        DriveConstants.kFrontRightDriveEncoderReversed, 
        DriveConstants.kFrontRightTurningEncoderReversed,
        CanIDs.kFrontRightAbsoluteEncoderPort, 
        DriveConstants.kFrontRightAbsoluteEncoderOffset);

    private final SwerveModule backLeft = new SwerveModule(
        CanIDs.kBackLeftDriveCanID, 
        CanIDs.kBackLeftTurningCanID, 
        DriveConstants.kBackLeftDriveEncoderReversed, 
        DriveConstants.kBackLeftTurningEncoderReversed,
        CanIDs.kBackLeftAbsoluteEncoderPort, 
        DriveConstants.kBackLeftAbsoluteEncoderOffset);

    private final SwerveModule backRight = new SwerveModule(
        CanIDs.kBackRightDriveCanID, 
        CanIDs.kBackRightTurningCanID, 
        DriveConstants.kBackRightDriveEncoderReversed, 
        DriveConstants.kBackRightTurningEncoderReversed,
        CanIDs.kBackRightAbsoluteEncoderPort, 
        DriveConstants.kBackRightAbsoluteEncoderOffset);

    private final Pigeon2 gyro = new Pigeon2(CanIDs.kPigeonID);
        
    private Pose2d updatedPose = new Pose2d();

    private final Field2d m_field = new Field2d();

    private double changeRule(double val){
        return 360 - val;
    }

    public double getHeading(){ // Right Hand Rule with Offset
        
        double gyroAngle = Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 360);
        return gyroAngle;
    }

    public Rotation2d getRotation2d(){ 

        return Rotation2d.fromDegrees(getHeading());
    }

    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
        DriveConstants.SwerveKinematics, 
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
        DriveConstants.SwerveKinematics,
        getRotation2d(),
        getPositions(),
        getOdometerPose());

    public Pose2d getPose(){
        return poseEstimator.getEstimatedPosition();
    }

    private boolean hasTags(){
        return visionSubsystem.getRobotPoseFromFrontCam().isPresent() || visionSubsystem.getRobotPoseFromLeftCam().isPresent() || visionSubsystem.getRobotPoseFromRightCam().isPresent();
    }
    private Pose2d updatePoseWithVision(){

        poseEstimator.update(getRotation2d(), getPositions());
        if (this.hasTags()) {
            SmartDashboard.putBoolean("Has Tags", true);
        }else{
            SmartDashboard.putBoolean("Has Tags", false);
        }
        
        Optional<EstimatedRobotPose> visionPoseFront = visionSubsystem.getRobotPoseFromFrontCam();
        Optional<EstimatedRobotPose> visionPoseLeft = visionSubsystem.getRobotPoseFromLeftCam();
        Optional<EstimatedRobotPose> visionPoseRight = visionSubsystem.getRobotPoseFromRightCam();

        if (visionPoseFront.isPresent()){
            SmartDashboard.putBoolean("Is Present", true);
            System.out.println("HAS VISION");

        }else{
              SmartDashboard.putBoolean("Is Present", false);
        }
        // Add vision measurements
        if (!visionPoseFront.isEmpty()){
            SmartDashboard.putNumber("VISION X", visionPoseFront.get().estimatedPose.toPose2d().getX());
            SmartDashboard.putNumber("VISION Y", visionPoseFront.get().estimatedPose.toPose2d().getY());
            SmartDashboard.putNumber("VISION ROTATION", visionPoseFront.get().estimatedPose.toPose2d().getRotation().getDegrees());
            poseEstimator.addVisionMeasurement(visionPoseFront.get().estimatedPose.toPose2d(), visionPoseFront.get().timestampSeconds);
        }
        if (!visionPoseLeft.isEmpty()){
            poseEstimator.addVisionMeasurement(visionPoseLeft.get().estimatedPose.toPose2d(), visionPoseLeft.get().timestampSeconds);
        }
        if (!visionPoseRight.isEmpty()){
            poseEstimator.addVisionMeasurement(visionPoseRight.get().estimatedPose.toPose2d(), visionPoseRight.get().timestampSeconds);
        }
        
       
        return poseEstimator.getEstimatedPosition();
    }

    public SwerveDriveSubsystem(VisionSubsystem visionSubsystem) {
        //Delay reset of navx for proper initialization

        this.gyroZero = 0;
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
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRelativeSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(2.5, 0.0, 0.0), // Translation PID constants
                new PIDConstants(2.5, 0.0, 0.0), // Rotation PID constants
                4.5, // Max module speed, in m/s
                Units.inchesToMeters(17.25), // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
  
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                  return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;}, // Whether to flip the path
            this // Reference to this subsystem to set requirements
        );
    }

    private boolean successZeroHeading = false;

    public void initZeroHeading() {
        gyro.reset();
        gyro.setYaw(180);
    }
    public void zeroHeading() {
        gyro.reset();
        
        // Optional<EstimatedRobotPose> currentPose = visionSubsystem.getRobotPoseFieldRelative(); 
        // if (currentPose.isEmpty() == false){
        
        //     double angle = (currentPose.get().estimatedPose.toPose2d().getRotation().getDegrees());  // Check make sure this is signed
        //     this.gyroZero = angle;
        //     this.gyro.reset();
        //     System.out.println("*******-------- NAVX VISION SUCCESS --------*******" + angle);
        //     successZeroHeading = true;
        // }//
        
    }

    

    private ChassisSpeeds getChassisSpeeds(){
        return DriveConstants.SwerveKinematics.toChassisSpeeds(
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
        System.out.println("**RESET ODOMETERY TO THE PRESET STARTING POSE**");
        poseEstimator.resetPosition(getRotation2d(), getPositions(), pose);
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
        this.updatedPose = this.updatePoseWithVision();
        m_field.setRobotPose(this.updatedPose);
        SmartDashboard.putData("Field", m_field);
        SmartDashboard.putNumber("Heading", getHeading());
        // SmartDashboard.putNumber("FLAbsolute", getFLAbsolutePosition());
        // SmartDashboard.putNumber("FRAbsolute", getFRAbsolutePosition());
        // SmartDashboard.putNumber("BLAbsolute", getBLAbsolutePosition());
        // SmartDashboard.putNumber("BRAbsolute", getBRAbsolutePosition());

        // SmartDashboard.putNumber("Timer", Timer.getFPGATimestamp());
    }
    
    public void driveRelativeSpeeds(ChassisSpeeds relativeSpeeds){
        SwerveModuleState[] states = DriveConstants.SwerveKinematics.toSwerveModuleStates(relativeSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        // SmartDashboard.putString("StateFL", states[0].toString());
        
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
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
}