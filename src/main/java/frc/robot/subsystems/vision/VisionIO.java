package frc.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.LimelightHelpers.RawFiducial;

@Logged
public interface VisionIO {

    @Logged
    public class VisionIOInputs {
        final RawFiducial[] emptyFiducials = new RawFiducial[0];

        boolean scoringCameraConnected = false;
        PoseEstimate scoringPoseEstimate = null;
        RawFiducial[] scoringFiducials = emptyFiducials;
    }

    public void update(VisionIOInputs inputs);

    public default void simulationPeriodic() {}
}