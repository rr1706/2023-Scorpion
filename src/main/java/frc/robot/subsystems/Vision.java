package frc.robot.subsystems;

import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

import frc.robot.Constants.VisionConstants;

public class Vision {
    private ArrayList<Pair<PhotonCamera, Transform3d>> m_cameras;
    private RobotPoseEstimator poseEstimator;

    public Vision() {

    }

    public Pair<Pose3d, Double> getEstimatedPose() {
        return null;
    }

    /**
     * Add a camera for use with photonvision
     * 
     * @param name Name of the camera
     * @param pose Pose of the camera with the bottom of the robot as the center
     */
    public void addCamera(String name, Pose3d pose) {
        m_cameras.add(new Pair(new PhotonCamera(name), pose));
        poseEstimator = new RobotPoseEstimator(VisionConstants.kTags, null, m_cameras);
    }

}
