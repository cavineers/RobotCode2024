package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

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

    private PhotonCamera camera;
    private AprilTagFieldLayout aprilTagFieldLayout;
    private Transform3d robotToCam;

    

    // Construct PhotonPoseEstimator
    PhotonPoseEstimator photonPoseEstimator;
    public VisionSubsystem() {
        camera = new PhotonCamera("PhotonCam1");
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {}
        robotToCam = new Transform3d(new Translation3d(0, -0.13, 0), new Rotation3d(0,0, 2.966)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCam);
        
    }

    public Optional<EstimatedRobotPose> getRobotPoseFieldRelative(){
        //photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        //return photonPoseEstimator.update(); 
        // if testing without april tags set up
        return Optional.empty();
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
    
    public void periodic() {
        
    }
}