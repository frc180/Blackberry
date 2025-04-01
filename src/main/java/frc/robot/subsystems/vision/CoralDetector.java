package frc.robot.subsystems.vision;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.util.LimelightHelpers.RawDetection;

public interface CoralDetector {

    public static final double MARGIN = 0.1; // was .125, .15
    
    public Pose2d getCoralPose(Pose2d robotPose, RawDetection[] detections);

    public default void reset() {};

    public static boolean isValid(Pose2d coralPose) {
        // Reject coral detections that are outside the field 
        double currMargin = RobotState.isAutonomous() ? MARGIN : 0;
        if (coralPose.getX() < currMargin || 
            coralPose.getX() > FlippingUtil.fieldSizeX - currMargin || 
            coralPose.getY() < currMargin || 
            coralPose.getY() > FlippingUtil.fieldSizeY - currMargin) {
           return false;
        }

        return true;
    }
}
