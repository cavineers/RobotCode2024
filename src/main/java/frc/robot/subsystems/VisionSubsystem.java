package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.proto.Photon;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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
    private Transform3d robotToCam;

    private boolean visionEnabled = false;

    

    // Construct PhotonPoseEstimator
    PhotonPoseEstimator photonPoseEstimatorFront;
    PhotonPoseEstimator photonPoseEstimatorLeft;
    PhotonPoseEstimator photonPoseEstimatorRight;
    public VisionSubsystem() {
        if (visionEnabled){
        cameraFront = new PhotonCamera("CameraFront");
        cameraLeft = new PhotonCamera("CameraLeft");
        cameraRight = new PhotonCamera("CameraRight");

        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {}
        robotToCam = new Transform3d(new Translation3d(0, -0.13, 0), new Rotation3d(0,0, 0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.        
        photonPoseEstimatorFront = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraFront, robotToCam);
        photonPoseEstimatorRight = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraRight, robotToCam);
        photonPoseEstimatorLeft = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraLeft, robotToCam);


        // If no tags are found use the most confident tag reading
        photonPoseEstimatorFront.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        photonPoseEstimatorRight.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        photonPoseEstimatorLeft.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        }
    }
    /**
     * Gets the robot pose from the front camera
     * @return Optional<EstimatedRobotPose> The robot pose from the front camera
     */
    public Optional<EstimatedRobotPose> getRobotPoseFromFrontCam(){

        if (!this.visionEnabled)
            return Optional.empty();

        return photonPoseEstimatorFront.update(); 
    }
    /**
     * Gets the robot pose from the left camera
     * @return Optional<EstimatedRobotPose> The robot pose from the left camera
     */
    public Optional<EstimatedRobotPose> getRobotPoseFromLeftCam(){

        if (!this.visionEnabled)
            return Optional.empty();

        return photonPoseEstimatorLeft.update(); 
    }
    /**
     * Gets the robot pose from the right camera
     * @return Optional<EstimatedRobotPose> The robot pose from the right camera
     */
    public Optional<EstimatedRobotPose> getRobotPoseFromRightCam(){

        if (!this.visionEnabled)
            return Optional.empty();

        return photonPoseEstimatorRight.update(); 
    }
    

     /**
     * @return double The X distance from you alliances speaker
     *  Always Positive
     */

    public double getDistanceFromSpeaker(){
        // Check alliance
        Pose2d currentPose = Robot.m_robotContainer.getSwerveSubsystem().getPose();
        if (!DriverStation.getAlliance().isPresent())
            return 0;

        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
            return Math.abs((currentPose.getX() - Constants.VisionConstants.blueSpeakerX));
        }
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
            return Math.abs((currentPose.getX() - Constants.VisionConstants.redSpeakerX));
        }
        return 0;
    }

    public double GetXDistanceSpeaker() {
        
        return 0;
    }
    
    public void periodic() {
        if (visionEnabled){
        SmartDashboard.putNumber("Distance from Speaker", getDistanceFromSpeaker());
        }
    }
}