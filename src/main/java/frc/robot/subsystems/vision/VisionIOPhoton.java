package frc.robot.subsystems.vision;

import com.spamrobotics.vision.SimCamera;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

public class VisionIOPhoton implements VisionIO {
    final SimCamera frontCamera;

    public VisionIOPhoton(AprilTagFieldLayout apriltagLayout) {
        frontCamera = new SimCamera("front", VisionSubsystem.ROBOT_TO_FRONT_CAMERA, apriltagLayout);
    }

    @Override
    public void update(VisionIOInputs inputs) {
        inputs.frontCameraConnected = true;
        inputs.frontPoseEstimate = frontCamera.getPoseEstimate();
        inputs.frontFiducials = frontCamera.getRawFiducials();
        if (inputs.frontPoseEstimate != null) inputs.frontPose = inputs.frontPoseEstimate.pose;
    }

    @Override
    public void simulationPeriodic() {
        frontCamera.update();
    }
}
