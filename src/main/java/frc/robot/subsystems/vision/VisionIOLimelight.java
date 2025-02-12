package frc.robot.subsystems.vision;

import com.spamrobotics.vision.LimelightStatus;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;

public class VisionIOLimelight implements VisionIO {

    private static final String SCORING_LIMELIGHT = "limelight";
    private static final String FRONT_LIMEIGHT = "front-limelight";
    private static final String BACK_LIMEIGHT = "back-limelight";
    private final LimelightStatus scoringLimelightStatus;
    private final LimelightStatus frontLimelightStatus;
    private final LimelightStatus backLimelightStatus;

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

        inputs.scoringFiducials = LimelightHelpers.getRawFiducials(SCORING_LIMELIGHT);
        inputs.backDetections = LimelightHelpers.getRawDetections(BACK_LIMEIGHT);
        
        if (Robot.isSimulation() && RobotContainer.MAPLESIM) {
            inputs.scoringPoseEstimate = simPoseEstimate;
            inputs.frontPoseEstimate = simPoseEstimate;
        } else {
            inputs.scoringPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(SCORING_LIMELIGHT);
            inputs.frontPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(FRONT_LIMEIGHT);
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
}
