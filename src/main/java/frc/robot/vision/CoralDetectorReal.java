package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.LimelightHelpers.RawDetection;

public class CoralDetectorReal implements CoralDetector {

    @Override
    public Pose2d getCoralPose(Pose2d robotPose, RawDetection[] detections) {
        // TODO: Convert RawDetections to a Pose2d of the best coral we see.
        // First we need to figure out the distance and angle from the robot to the coral.
        // Once we have that, we can convert it to a Pose2d.
        return null;
    }
}
