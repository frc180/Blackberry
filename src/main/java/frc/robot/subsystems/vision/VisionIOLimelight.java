package frc.robot.subsystems.vision;

import com.spamrobotics.vision.LimelightStatus;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;

public class VisionIOLimelight implements VisionIO {

    private static final String SCORING_LIMELIGHT = "limelight";
    private static final String FRONT_LIMEIGHT = "limelight-front";
    private static final String BACK_LIMEIGHT = "limelight-back";
    private final LimelightStatus scoringLimelightStatus;
    private final LimelightStatus frontLimelightStatus;
    private final LimelightStatus backLimelightStatus;

    private final int APRILTAG_PIPELINE = 0;
    private final int VIEWFINDER_PIPELINE = 1;

    private final PoseEstimate simPoseEstimate = new PoseEstimate();

    public VisionIOLimelight() {
        scoringLimelightStatus = new LimelightStatus(SCORING_LIMELIGHT);
        frontLimelightStatus = new LimelightStatus(FRONT_LIMEIGHT);
        backLimelightStatus = new LimelightStatus(BACK_LIMEIGHT);
    }

    @Override
    public void update(VisionIOInputs inputs) {
        scoringLimelightStatus.update();
        frontLimelightStatus.update();
        backLimelightStatus.update();

        inputs.scoringCameraConnected = scoringLimelightStatus.isConnected();
        inputs.frontCameraConnected = frontLimelightStatus.isConnected();
        inputs.backCameraConnected = backLimelightStatus.isConnected();

        // LimelightHelpers.setPipelineIndex(SCORING_LIMELIGHT, APRILTAG_PIPELINE);
        // LimelightHelpers.setPipelineIndex(FRONT_LIMEIGHT, APRILTAG_PIPELINE);

        // double throttle = 0;
        // if (RobotState.isDisabled()) throttle = 200;
        
        // LimelightHelpers.setLimelightNTDouble("throttle_set", SCORING_LIMELIGHT, throttle);
        // LimelightHelpers.setLimelightNTDouble("throttle_set", FRONT_LIMEIGHT, throttle);

        // TODO: See if we need to call this every loop
        setLimelightPosition(SCORING_LIMELIGHT, VisionSubsystem.ROBOT_TO_SCORING_CAMERA);
        setLimelightPosition(FRONT_LIMEIGHT, VisionSubsystem.ROBOT_TO_FRONT_CAMERA);

        inputs.scoringFiducials = LimelightHelpers.getRawFiducials(SCORING_LIMELIGHT);
        inputs.frontFiducials = LimelightHelpers.getRawFiducials(FRONT_LIMEIGHT);
        inputs.backDetections = LimelightHelpers.getRawDetections(BACK_LIMEIGHT);
        inputs.backTimestamp = Timer.getFPGATimestamp() - getLatencySeconds(BACK_LIMEIGHT);
        
        if (Robot.isSimulation() && RobotContainer.MAPLESIM) {
            inputs.scoringPoseEstimate = simPoseEstimate;
            inputs.frontPoseEstimate = simPoseEstimate;
        } else {
            inputs.scoringPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(SCORING_LIMELIGHT);
            inputs.frontPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(FRONT_LIMEIGHT);
        }

        if (inputs.scoringPoseEstimate != null) {
            inputs.scoringTimestamp = inputs.scoringPoseEstimate.timestampSeconds;
        }

        // fps, cpu temp, ram usage, temp
        double[] hw = LimelightHelpers.getLimelightNTDoubleArray(SCORING_LIMELIGHT, "hw");
        if (hw.length > 3) {
            inputs.scoringCPUTemp = hw[1];
            inputs.scoringTemp = hw[3];
        } else {
            inputs.scoringCPUTemp = - 1;
            inputs.scoringTemp = -1;
        }

        hw = LimelightHelpers.getLimelightNTDoubleArray(FRONT_LIMEIGHT, "hw");
        if (hw.length > 3) {
            inputs.frontTemp = hw[3];
        } else {
            inputs.frontTemp = -1;
        }
    }

    @Override
    public void simulationPeriodic() {
        // If we're running in simulation, we can feed constant perfect vision data to the robot 
        // to stop the odometry from diverging from the real robot position if we're using a physics simulation (like MapleSim).
        // To simulate realistic vision input using Apriltags, use {@link VisionIOPhoton} instead.
        simPoseEstimate.pose = RobotContainer.instance.drivetrain.getSimPose();
        simPoseEstimate.timestampSeconds = Timer.getFPGATimestamp();
        simPoseEstimate.tagCount = 1;
        simPoseEstimate.avgTagDist = 2;
        simPoseEstimate.avgTagArea = 99;
    }

    private double getLatencySeconds(String limelightName) {
        double latency = LimelightHelpers.getLatency_Capture(limelightName) + LimelightHelpers.getLatency_Pipeline(limelightName);
        return latency * 0.001;
    }

    private void setLimelightPosition(String limelightName, Transform3d transform) {
        LimelightHelpers.setCameraPose_RobotSpace(
            limelightName,
            transform.getX(), // forward
            -transform.getY(), // side
            transform.getZ(), // up
            -Units.radiansToDegrees(transform.getRotation().getX()), // roll
            -Units.radiansToDegrees(transform.getRotation().getY()), // pitch
            -Units.radiansToDegrees(transform.getRotation().getZ()) // yaw
        );
    }
}
