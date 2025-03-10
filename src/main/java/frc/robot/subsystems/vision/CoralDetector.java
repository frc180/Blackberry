package frc.robot.subsystems.vision;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.LimelightHelpers.RawDetection;

public interface CoralDetector {

    public static final double MARGIN = 0.1;
    
    public Pose2d getCoralPose(Pose2d robotPose, RawDetection[] detections);

    public default void reset() {};

    public static boolean isValid(Pose2d coralPose) {
        // Reject coral detections that are outside the field 
        if (coralPose.getX() < MARGIN || 
            coralPose.getX() > FlippingUtil.fieldSizeX - MARGIN || 
            coralPose.getY() < MARGIN || 
            coralPose.getY() > FlippingUtil.fieldSizeY - MARGIN) {
           return false;
        }

        return true;
    }
}
