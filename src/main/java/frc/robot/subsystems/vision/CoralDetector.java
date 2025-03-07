package frc.robot.subsystems.vision;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.LimelightHelpers.RawDetection;

public interface CoralDetector {
    
    public Pose2d getCoralPose(Pose2d robotPose, RawDetection[] detections);

    public static boolean isValid(Pose2d coralPose) {
        // Reject coral detections that are outside the field 
        if (coralPose.getX() < 0 || coralPose.getX() > FlippingUtil.fieldSizeX || coralPose.getY() < 0 || coralPose.getY() > FlippingUtil.fieldSizeY) {
           return false;
        }

        return true;
    }
}
