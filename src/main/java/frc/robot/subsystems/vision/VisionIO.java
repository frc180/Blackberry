package frc.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.LimelightHelpers.RawDetection;
import frc.robot.util.LimelightHelpers.RawFiducial;

@Logged
public interface VisionIO {

    @Logged
    public class VisionIOInputs {
        final RawFiducial[] emptyFiducials = new RawFiducial[0];
        final RawDetection[] emptyDetections = new RawDetection[0];

        boolean frontCameraConnected = false;
        PoseEstimate frontPoseEstimate = null;
        Pose2d frontPose = null;
        RawFiducial[] frontFiducials = emptyFiducials;
        double frontTemp = 0;
    }

    public void update(VisionIOInputs inputs);

    public default void simulationPeriodic() {}
}