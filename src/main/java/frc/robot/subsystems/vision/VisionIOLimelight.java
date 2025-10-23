package frc.robot.subsystems.vision;

import com.spamrobotics.vision.LimelightStatus;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;

public class VisionIOLimelight implements VisionIO {

    private static final String FRONT_LIMEIGHT = "limelight";

    private final LimelightStatus frontLimelightStatus;
    private final PoseEstimate simPoseEstimate = new PoseEstimate();

    private final double[] blueReefTags;
    private final double[] redReefTags;
    private final double[] allReefTags;

    private double lastSettingsUpdate = -1;

    public VisionIOLimelight() {
        frontLimelightStatus = new LimelightStatus(FRONT_LIMEIGHT);

        blueReefTags = new double[VisionSubsystem.blueReefTags.size()];
        for (int i = 0; i < blueReefTags.length; i++) {
            blueReefTags[i] = VisionSubsystem.blueReefTags.get(i);
        }
        
        redReefTags = new double[VisionSubsystem.redReefTags.size()];
        for (int i = 0; i < redReefTags.length; i++) {
            redReefTags[i] = VisionSubsystem.redReefTags.get(i);
        }

        allReefTags = new double[VisionSubsystem.allReefTags.size()];
        for (int i = 0; i < allReefTags.length; i++) {
            allReefTags[i] = VisionSubsystem.allReefTags.get(i);
        }
    }

    @Override
    public void update(VisionIOInputs inputs) {
        frontLimelightStatus.update();

        inputs.frontCameraConnected = frontLimelightStatus.isConnected();

        double time = Timer.getFPGATimestamp();
        if (lastSettingsUpdate == -1 || time - lastSettingsUpdate > 5) {
            setLimelightPosition(FRONT_LIMEIGHT, VisionSubsystem.ROBOT_TO_FRONT_CAMERA);

            double[] validTags = allReefTags;
            setValidTags(FRONT_LIMEIGHT, validTags);

            lastSettingsUpdate = time;
        }


        inputs.frontFiducials = LimelightHelpers.getRawFiducials(FRONT_LIMEIGHT);
        
        if (Robot.isSimulation()) {
            inputs.frontPoseEstimate = simPoseEstimate;
        } else {
            inputs.frontPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(FRONT_LIMEIGHT);
        }
        if (inputs.frontPoseEstimate != null) inputs.frontPose = inputs.frontPoseEstimate.pose;
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

    private void setValidTags(String limelightName, double[] validTags) {
        LimelightHelpers.setLimelightNTDoubleArray(limelightName, "fiducial_id_filters_set", validTags);
    }
}
