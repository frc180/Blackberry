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

    private static final String SCORING_LIMELIGHT = "limelight";
    private static final String FRONT_LIMEIGHT = "limelight-front";
    private static final String BACK_LIMEIGHT = "limelight-back";

    private static final int IMU_EXTERNALIMU_ONLY = 0;   // 0 - Use external IMU yaw submitted via SetRobotOrientation() for MT2 localization. The internal IMU is ignored entirely.
    private static final int IMU_MATCH_EXTERNALIMU = 1;  // 1 - Use external IMU yaw submitted via SetRobotOrientation(), and configure the LL4 internal IMU’s fused yaw to match the submitted yaw value.
    private static final int IMU_INTERNALIMU_ONLY = 2;   // 2 - Use internal IMU for MT2 localization. External imu data is ignored entirely
    private static final int IMU_ASSIST_MT1 = 3;         // 3 - The internal IMU will utilize filtered MT1 yaw estimates for continuous heading correction
    private static final int IMU_ASSIST_EXTERNALIMU = 4; // 4 - The internal IMU will utilize the external IMU for continuous heading correction

    private final LimelightStatus scoringLimelightStatus;
    private final LimelightStatus frontLimelightStatus;
    private final LimelightStatus backLimelightStatus;
    private final PoseEstimate simPoseEstimate = new PoseEstimate();

    private final double[] blueReefTags;
    private final double[] redReefTags;
    private final double[] allReefTags;

    private final double[] blueBargeTags;
    private final double[] redBargeTags;

    private int frontCameraImuMode = IMU_ASSIST_EXTERNALIMU;
    private boolean wasEnabled = false;
    private boolean bargeMode = false;
    private double lastSettingsUpdate = -1;

    public VisionIOLimelight() {
        scoringLimelightStatus = new LimelightStatus(SCORING_LIMELIGHT);
        frontLimelightStatus = new LimelightStatus(FRONT_LIMEIGHT);
        backLimelightStatus = new LimelightStatus(BACK_LIMEIGHT);

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

        // 4 & 5 are on the red side, 14 & 15 are on the blue side. 3 & 16 processor
        blueBargeTags = new double[] { 4, 5, 14, 15, 3, 16 };
        redBargeTags = blueBargeTags;
    }

    @Override
    public void update(VisionIOInputs inputs) {
        scoringLimelightStatus.update();
        frontLimelightStatus.update();
        backLimelightStatus.update();

        inputs.scoringCameraConnected = scoringLimelightStatus.isConnected();
        inputs.frontCameraConnected = frontLimelightStatus.isConnected();
        inputs.backCameraConnected = backLimelightStatus.isConnected();

        double time = Timer.getFPGATimestamp();
        if (lastSettingsUpdate == -1 || time - lastSettingsUpdate > 5) {
            setLimelightPosition(SCORING_LIMELIGHT, VisionSubsystem.ROBOT_TO_SCORING_CAMERA);
            setLimelightPosition(FRONT_LIMEIGHT, VisionSubsystem.ROBOT_TO_FRONT_CAMERA);

            // double[] validTags = Robot.isBlue() ? blueReefTags : redReefTags;
            double[] validTags = allReefTags;
            setValidTags(SCORING_LIMELIGHT, validTags);
            setValidTags(FRONT_LIMEIGHT, validTags);

            lastSettingsUpdate = time;
        }

        // double[] validTags;
        // if (bargeMode) {
        //     validTags = Robot.isBlue() ? blueBargeTags : redBargeTags;
        // } else {
        //     validTags = Robot.isBlue() ? blueReefTags : redReefTags;
        // }
        // setValidTags(SCORING_LIMELIGHT, validTags);
        // setValidTags(FRONT_LIMEIGHT, validTags);
 
        // EXPERIMENT: All of this is irrelevant since we don't use MegaTag2
        // if (RobotState.isEnabled()) {
        //     frontCameraImuMode = IMU_ASSIST_EXTERNALIMU;
        // } else {
        //     frontCameraImuMode = IMU_MATCH_EXTERNALIMU;
        // }
        // LimelightHelpers.SetIMUMode(FRONT_LIMEIGHT, frontCameraImuMode);

        // DrivetrainSubsystem drivetrain = RobotContainer.instance.drivetrain;
        // double headingDegrees = drivetrain.getGyroscopeDegrees();
        // double yawRate = drivetrain.getGyroscopeRate();

        // if (Robot.isRed()) headingDegrees += 180;
        // LimelightHelpers.SetRobotOrientation(FRONT_LIMEIGHT, headingDegrees, yawRate, 0, 0, 0, 0);

        inputs.scoringFiducials = LimelightHelpers.getRawFiducials(SCORING_LIMELIGHT);
        inputs.frontFiducials = LimelightHelpers.getRawFiducials(FRONT_LIMEIGHT);
        inputs.backDetections = LimelightHelpers.getRawDetections(BACK_LIMEIGHT);
        if (inputs.backCameraConnected) {
            inputs.backTimestamp = Timer.getFPGATimestamp() - getLatencySeconds(BACK_LIMEIGHT);
        }
        
        if (Robot.isSimulation() && RobotContainer.MAPLESIM) {
            inputs.scoringPoseEstimate = simPoseEstimate;
            inputs.frontPoseEstimate = simPoseEstimate;
        } else {
            inputs.scoringPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(SCORING_LIMELIGHT);
            inputs.frontPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(FRONT_LIMEIGHT);
            // inputs.frontPoseEstimateMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(FRONT_LIMEIGHT);
        }

        if (inputs.scoringPoseEstimate != null) {
            inputs.scoringTimestamp = inputs.scoringPoseEstimate.timestampSeconds;
        }

        // fps, cpu temp, ram usage, temp
        // double[] hw = LimelightHelpers.getLimelightNTDoubleArray(SCORING_LIMELIGHT, "hw");
        // if (hw.length > 3) {
        //     inputs.scoringCPUTemp = hw[1];
        //     inputs.scoringTemp = hw[3];
        // } else {
        //     inputs.scoringCPUTemp = - 1;
        //     inputs.scoringTemp = -1;
        // }

        // hw = LimelightHelpers.getLimelightNTDoubleArray(FRONT_LIMEIGHT, "hw");
        // if (hw.length > 3) {
        //     inputs.frontTemp = hw[3];
        // } else {
        //     inputs.frontTemp = -1;
        // }
    }

    
    @Override
    public void setBargeMode(boolean bargeModeEnabled) {
        this.bargeMode = bargeModeEnabled;
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
