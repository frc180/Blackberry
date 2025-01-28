package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.LimelightHelpers.RawDetection;

public interface CoralDetector {
    
    public Pose2d getCoralPose(Pose2d robotPose, RawDetection[] detections);
}
