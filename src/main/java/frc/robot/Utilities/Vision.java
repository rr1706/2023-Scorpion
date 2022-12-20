package frc.robot.Utilities;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;

public class Vision {
    public PhotonCamera m_camera;
    public List<PhotonTrackedTarget> targets;

    public Vision(PhotonCamera camera) {
        m_camera = camera;
    }

    public List<PhotonTrackedTarget> getBestTargets(int n) {
        if (targets.size() < n) {
            return targets;
        } else if (n != 0) {
            return targets.stream().limit(n).collect(Collectors.toList());
        } else {
            return new ArrayList<PhotonTrackedTarget>();
        }
    }

    public Pose2d getPose() {

        targets = m_camera.getLatestResult().getTargets();

        return PhotonUtils.estimateFieldToRobot(0, 0, 0, 0, null, null, getPose(), null);
    }

}
