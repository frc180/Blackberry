package frc.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.LimelightHelpers.RawDetection;
import frc.robot.util.LimelightHelpers.RawFiducial;

@Logged
public interface VisionIO {

    @Logged
    public class VisionIOInputs {
        final RawFiducial[] emptyFiducials = new RawFiducial[0];
        final RawDetection[] emptyDetections = new RawDetection[0];

        boolean scoringCameraConnected = false;
        PoseEstimate scoringPoseEstimate = null;
        RawFiducial[] scoringFiducials = emptyFiducials;

        boolean frontCameraConnected = false;
        PoseEstimate frontPoseEstimate = null;

        boolean backCameraConnected = false;
        RawDetection[] backDetections = emptyDetections;
    }

    public void update(VisionIOInputs inputs);

    public default void simulationPeriodic() {}
}