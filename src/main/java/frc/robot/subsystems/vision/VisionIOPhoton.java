package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.LimelightHelpers.RawFiducial;

public class VisionIOPhoton implements VisionIO {
    PhotonCamera scoringCamera;
    Transform3d robotToCamera = new Transform3d();
    PhotonPoseEstimator photonPoseEstimator;

    VisionSystemSim visionSim = null;
    PhotonCameraSim cameraSim = null;

    public VisionIOPhoton(AprilTagFieldLayout apriltagLayout) {
        scoringCamera = new PhotonCamera("scoring");
        photonPoseEstimator = new PhotonPoseEstimator(apriltagLayout, PoseStrategy.LOWEST_AMBIGUITY, robotToCamera);

        // Everything past this point is simulation only
        if (Robot.isReal()) return;

        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(apriltagLayout);

        // These have not been adjusted yet, just copied from the PhotonVision example
        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
        cameraProp.setCalibError(0.35, 0.10);
        cameraProp.setFPS(80);
        cameraProp.setAvgLatencyMs(50);
        cameraProp.setLatencyStdDevMs(15);
        // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
        // targets.
        cameraSim = new PhotonCameraSim(scoringCamera, cameraProp);
        // Add the simulated camera to view the targets on this simulated field.
        visionSim.addCamera(cameraSim, robotToCamera);

        cameraSim.enableDrawWireframe(true);
    }

    @Override
    public void update(VisionIOInputs inputs) {
        inputs.scoringCameraConnected = true;

        PhotonPipelineResult latestResult = null;
        EstimatedRobotPose latestEstimate = null;

        for (PhotonPipelineResult result : scoringCamera.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> poseEst = photonPoseEstimator.update(result);
            if (poseEst.isPresent()) {
                latestEstimate = poseEst.get();
                latestResult = result;
            }
        }
        List<PhotonTrackedTarget> targets = latestResult != null ? latestResult.getTargets() : null;

        if (latestEstimate != null) {
            // Convert the photonvision estimate to a Limelight PoseEstimate
            PoseEstimate est = new PoseEstimate();
            est.pose = latestEstimate.estimatedPose.toPose2d();
            est.timestampSeconds = latestEstimate.timestampSeconds;
            est.tagCount = targets.size();
            est.avgTagDist = targets.stream().mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getDistance(Translation3d.kZero)).average().orElse(0);
            est.avgTagArea = 99;//targets.stream().mapToDouble(t -> t.area).average().orElse(0);
            inputs.scoringPoseEstimate = est;
        } else {
            inputs.scoringPoseEstimate = null;
        }

        if (targets != null) {
            RawFiducial[] fiducials = new RawFiducial[targets.size()];
            for (int i = 0; i < targets.size(); i++) {
                PhotonTrackedTarget target = targets.get(i);
                double dist = target.getBestCameraToTarget().getTranslation().getDistance(Translation3d.kZero);

                fiducials[i] = new RawFiducial(
                    target.getFiducialId(),
                    0,
                    0,
                    target.area,
                    dist,
                    dist,
                    target.poseAmbiguity
                );
            }
            inputs.scoringFiducials = fiducials;
        } else {
            inputs.scoringFiducials = inputs.emptyFiducials;
        }
    }

    @Override
    public void simulationPeriodic() {
        visionSim.update(RobotContainer.instance.drivetrain.getSimPose());
    }
}
