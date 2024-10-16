package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class VisionSubsystem extends SubsystemBase {

    private PhotonCamera cameraFront;
    private PhotonCamera cameraLeft;
    private PhotonCamera cameraRight;

    private AprilTagFieldLayout aprilTagFieldLayout;
    private Transform3d robotToCamFront;

    private boolean visionEnabled = true;
    private boolean autoShoot = false;
    private boolean autoRotate = true;
    

    // Construct PhotonPoseEstimator
    PhotonPoseEstimator photonPoseEstimatorFront;
    PhotonPoseEstimator photonPoseEstimatorLeft;
    PhotonPoseEstimator photonPoseEstimatorRight;
    public VisionSubsystem() {
        if (visionEnabled){
        cameraFront = new PhotonCamera("CameraFront");
        // cameraLeft = new PhotonCamera("CameraLeft");
        // cameraRight = new PhotonCamera("CameraRight");

        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {}
        robotToCamFront = new Transform3d(new Translation3d(Units.inchesToMeters(16),0 , 0.46), new Rotation3d(Units.degreesToRadians(180),Units.degreesToRadians(-14), 0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.        
        photonPoseEstimatorFront = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraFront, robotToCamFront);
        // photonPoseEstimatorRight = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraRight, robotToCam);
        // photonPoseEstimatorLeft = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraLeft, robotToCam);


        // If no tags are found use the most confident tag reading
        photonPoseEstimatorFront.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        // photonPoseEstimatorRight.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        // photonPoseEstimatorLeft.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        }
    }
    /**
     * Gets the robot pose from the front camera
     * @return Optional<EstimatedRobotPose> The robot pose from the front camera
     */
    public Optional<EstimatedRobotPose> getRobotPoseFromFrontCam(){

        if (!this.visionEnabled)
            return Optional.empty();

        Optional<EstimatedRobotPose> pose = photonPoseEstimatorFront.update();
        return pose;
    }
    /**
     * Gets the robot pose from the left camera
     * @return Optional<EstimatedRobotPose> The robot pose from the left camera
     */
    public Optional<EstimatedRobotPose> getRobotPoseFromLeftCam(){

        if (!this.visionEnabled)
            return Optional.empty();

        return photonPoseEstimatorFront.update(); 
    }
    /**
     * Gets the robot pose from the right camera
     * @return Optional<EstimatedRobotPose> The robot pose from the right camera
     */
    public Optional<EstimatedRobotPose> getRobotPoseFromRightCam(){

        if (!this.visionEnabled)
            return Optional.empty();

        return photonPoseEstimatorFront.update(); 
    }
    

     /**
     * @return Whether the robot is capable of performing the auto shoot command
     *
     */

    public boolean autoShootCapable(){
        return this.autoShoot;
    }

    /**
     * Gets the distance from the alliance's speaker, +x is to the right, -x to the left
     * @return The distance from the speaker, returns 0 if no alliance is present or some other issue occured
     */

    private double getXDistanceFromSpeaker(){
        // Check alliance
        Pose2d currentPose = Robot.m_robotContainer.getSwerveSubsystem().getPose();
        if (!DriverStation.getAlliance().isPresent()){
            this.autoShoot = false;
            return 0;
        }
        double x1 = currentPose.getX();
    
        double x2;
    
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
            x2 = Constants.VisionConstants.blueSpeakerX;
        } else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
            x2 = Constants.VisionConstants.redSpeakerX;
        } else {
            return 0;
        }
    
        return x2 - x1;
    }
    /**
     * Clarify with this.autoRotate to ensure that the bot is capable of performing the auto rotate.
     * @return returns the angle to speaker, returns 0 if no alliance is present or some other issue occured
     */
    public double getAngleToSpeaker(){
        return Units.radiansToDegrees(Math.atan2(getYDistanceFromSpeaker(), getXDistanceFromSpeaker()));
    }

    /**
     * Gets the distance from the alliance's speaker, +y is up, -y down
     * @return The distance from the speaker, returns 0 if no alliance is present or some other issue occured
     */
    private double getYDistanceFromSpeaker(){
        // Check alliance
        Pose2d currentPose = Robot.m_robotContainer.getSwerveSubsystem().getPose();
        if (!DriverStation.getAlliance().isPresent()){
            this.autoShoot = false;
            return 0;
        }
        double y1 = currentPose.getY();
    
        double y2;
    
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
            y2 = Constants.VisionConstants.blueSpeakerY;
        } else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
            y2 = Constants.VisionConstants.redSpeakerY;
        } else {
            this.autoShoot = false;
            return 0;

        }
        this.autoShoot = true;
    
        return y2 - y1;
    }
    


    /**
     * @return The distance (2d distance straight line) from the speaker (INCHES), always positive, returns 0 if no alliance is present or some other issue occured
     * Clarify the autoShootCapable() method to ensure that the bot is capable of performing the auto shoot.
     */
    public double getDistanceFromSpeaker(){
        // Check alliance
        Pose2d currentPose = Robot.m_robotContainer.getSwerveSubsystem().getPose();
        if (!DriverStation.getAlliance().isPresent()){
            this.autoShoot = false;
            System.out.println("ERROR: NO DRIVER STATION ALLIANCE ");

            return 0;
        }

        double x2, y2;
        x2 = getXDistanceFromSpeaker();
        y2 = getYDistanceFromSpeaker();

        double dist = Math.sqrt(Math.pow(x2, 2) + Math.pow(y2, 2));
        SmartDashboard.putNumber("Distance from Speaker", Units.metersToInches(dist));
        // if ((dist < Constants.VisionConstants.shootDistanceMinimum) || (dist > Constants.VisionConstants.shootDistanceMaximum)){
        //     this.autoShoot = false;
        //     return 0;
        
        // }
        
        this.autoShoot = true;
        return Units.metersToInches(dist);
    }
    
    public void periodic() {
        if (visionEnabled){
            // SmartDashboard.putNumber("Distance from Speaker", getDistanceFromSpeaker());
            getDistanceFromSpeaker();
            if (autoShootCapable()){
                SmartDashboard.putBoolean("AutoShoot Capable", true);
                SmartDashboard.putNumber("X DISTANCE TO TAG", getDistanceFromSpeaker());
            } else {
                SmartDashboard.putBoolean("AutoShoot Capable", false);
            }

            if (autoRotate){
                SmartDashboard.putBoolean("AutoRotate Capable", true);
                SmartDashboard.putNumber("Angle to Speaker", getAngleToSpeaker());
            } else {
                SmartDashboard.putBoolean("AutoRotate Capable", false);
                SmartDashboard.putNumber("Angle to Speaker", 0);
            }
            SmartDashboard.putNumber("Distance to Speaker", getDistanceFromSpeaker());

            


        }


    }
    
}