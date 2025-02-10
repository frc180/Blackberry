package frc.robot.subsystems.vision;

import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import com.ctre.phoenix6.Utils;
import java.util.Map.Entry;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.util.ReefProximity;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.LimelightHelpers.RawFiducial;
import frc.robot.vision.CoralDetector;
import frc.robot.vision.CoralDetectorReal;
import frc.robot.vision.CoralDetectorSim;

@Logged
public class VisionSubsystem extends SubsystemBase {

    public static final List<Integer> redReefTags = List.of(6,7,8,9,10,11);
    public static final List<Integer> blueReefTags = List.of(17, 18, 19, 20, 21, 22);
    public static final Transform3d ROBOT_TO_SCORING_CAMERA = new Transform3d(0.1, 0, 0.65, new Rotation3d(0, Units.degreesToRadians(35), 0));

    private VisionIO io;
    private VisionIOInputs inputs;

    public AprilTagFieldLayout aprilTagFieldLayout;

    private boolean canSeeReef = false;
    public int bestReefID = -1;

    public int lastReefID = -1;

    int[] fiducialArray = new int[0];

    private final ReefProximity reefProximity;
    private final CoralDetector coralDetector;

    private List<Integer> redTags = List.of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11);
    private List<Integer> blueTags = List.of(12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22);
    private int redProcessorTag = 3;
    private int blueProcessorTag = 16;

    private final Transform2d leftReefTransform = new Transform2d(0.55, -0.15, Rotation2d.fromDegrees(180));
    private final Transform2d rightReefTransform = new Transform2d(0.55, 0.15, Rotation2d.fromDegrees(180));

    private final Transform2d processorTransform = new Transform2d(0.55, 0.0, Rotation2d.fromDegrees(90));

    public final HashMap<Integer, Pose2d> leftReefHashMap = new HashMap<>();
    public final HashMap<Integer, Pose2d> rightReefHashMap = new HashMap<>();

    private Pose2d exampleLeft;
    private Pose2d exampleRight;
    private Pose2d coralPose = Pose2d.kZero;
    private boolean coralPoseValid = false;

    private boolean closestReefPoseValid = false;
    private Pose2d closestReefPose = Pose2d.kZero;

    final boolean megatag2Enabled = false;
    @NotLogged
    private Matrix<N3, N1> measureStdDev = MatBuilder.fill(Nat.N3(), Nat.N1(), 0.3, 0.3, 0.3);

    private Pose3d scoringCamera = Pose3d.kZero;
    private Pose2d scoringPoseEstimatePoseUnfiltered = Pose2d.kZero;
    private Pose2d scoringPoseEstimatePose = Pose2d.kZero;
    private double poseEstimateDiffX, poseEstimateDiffY, poseEstimateDiffTheta;

    private Pose2d redProcessorPose;
    private Pose2d blueProcessorPose;
    
    private Alert scoringCameraDisconnectedAlert = new Alert("Scoring Camera disconnected!", AlertType.kError);

    @SuppressWarnings("unused")
    public VisionSubsystem() {
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
        } catch (Exception e) {
            aprilTagFieldLayout = null;
        }

        inputs = new VisionIOInputs();
        // io = new VisionIOLimelight();
        io = new VisionIOPhoton(aprilTagFieldLayout);
        // if (Robot.isReal() || !RobotContainer.MAPLESIM) {
        //     io = new VisionIOLimelight();
        // } else {
        //     // PhotonVision has simulated apriltag odometry, which is needed in Maplesim
        //     // because odometry slippage is properly simulated, meaning we need to simulate
        //     // feeding in vision data to fix the odometry like we would in real life
        //     io = new VisionIOPhoton(aprilTagFieldLayout);
        // }

        //red reef tags
        for (int i = 6; i <= 11; i++) {
            leftReefHashMap.put(i, calculateReefPose(i, true));
            rightReefHashMap.put(i, calculateReefPose(i, false));
        }
        //left reef tags
        for (int i = 17; i <= 22; i++) {
            leftReefHashMap.put(i, calculateReefPose(i, true));
            rightReefHashMap.put(i, calculateReefPose(i, false));
        }

        //exampleLeft = leftReefHashMap.get(18);
        //exampleRight = rightReefHashMap.get(18);

        reefProximity = new ReefProximity(leftReefHashMap, rightReefHashMap);
        coralDetector = Robot.isReal() ? new CoralDetectorReal() : new CoralDetectorSim(4.0, true);

        redProcessorPose = calculateProcessorPose(false);
        blueProcessorPose = calculateProcessorPose(true);
    }

    @Override
    public void periodic() {
        io.update(inputs);

        scoringCameraDisconnectedAlert.set(!inputs.scoringCameraConnected);

        // Update odometry using Limelight in apriltag mode
        // scoringPoseEstimate = validatePoseEstimate(LimelightHelpers.getBotPoseEstimate_wpiBlue(SCORING_LIMELIGHT), 0);
        PoseEstimate scoringPoseEstimate = inputs.scoringPoseEstimate;
        scoringPoseEstimatePoseUnfiltered = scoringPoseEstimate != null ? scoringPoseEstimate.pose : Pose2d.kZero;
        scoringPoseEstimate = validatePoseEstimate(scoringPoseEstimate, 0);
        scoringPoseEstimatePose = scoringPoseEstimate != null ? scoringPoseEstimate.pose : Pose2d.kZero;
        
        Pose2d robotPose = null;
        if (scoringPoseEstimate != null) {
            RobotContainer.instance.drivetrain.addVisionMeasurement(
                scoringPoseEstimate.pose,
                Utils.fpgaToCurrentTime(scoringPoseEstimate.timestampSeconds)
            );
            // Calculate the difference between the updated robot pose and the scoring pose estimate, to get an idea
            // of how closely we
            robotPose = RobotContainer.instance.drivetrain.getPose();
            poseEstimateDiffX = robotPose.getX() - scoringPoseEstimate.pose.getX();
            poseEstimateDiffY = robotPose.getY() - scoringPoseEstimate.pose.getY();
            poseEstimateDiffTheta = robotPose.getRotation().getDegrees() - scoringPoseEstimate.pose.getRotation().getDegrees();
        }

        if (robotPose == null) robotPose = RobotContainer.instance.drivetrain.getPose();

        scoringCamera = new Pose3d(robotPose).transformBy(ROBOT_TO_SCORING_CAMERA);

        //check if robot can see the reef
        canSeeReef = reefVisible();

        //display on advantage scope where the robot thinks the scoring position should be
        exampleLeft = getReefPose(true);
        exampleRight = getReefPose(false);

        Entry<Integer, Pose2d> closestTagAndPose = reefProximity.closestReefPose(robotPose, Robot.isBlue());
        if (closestTagAndPose == null) {
            closestReefPose = Pose2d.kZero;
            closestReefPoseValid = false;
        } else {
            closestReefPose = closestTagAndPose.getValue();
            closestReefPoseValid = true;
        }

        // TODO: Pass in an array of RawDetections from VisionInputs instead of null
        coralPose = coralDetector.getCoralPose(robotPose, null);
        if (coralPose == null) {
            coralPose = Pose2d.kZero;
            coralPoseValid = false;
        } else {
            coralPoseValid = true;
        }

        // For now, we're just going to use the closest reef tag to the robot
        if (false && inputs.scoringCameraConnected) {
            fiducialArray = new int[inputs.scoringFiducials.length];
            for(int i = 0; i < inputs.scoringFiducials.length; i++) {
                fiducialArray[i] = inputs.scoringFiducials[i].id;
            }

            bestReefID = getReefTag(inputs.scoringFiducials);
        } else {
            // Simulate the vision system by selecting the closest reef tag to the robot position
            if (closestTagAndPose == null) {
                bestReefID = -1;
            } else {
                bestReefID = closestTagAndPose.getKey();
            }
        }

        if (bestReefID != -1) {
            lastReefID = bestReefID;
        }
    }

    
    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public int getReefTag(RawFiducial[] rawFiducial) {
        RawFiducial bestTag = null;
        List<Integer> reefTags;
        if (Robot.isBlue()) {
            reefTags = blueReefTags;
        } else {
            reefTags = redReefTags;
        }
        // for each tag detected, look for the IDs that specifically belong to a reef based on which alliance we are on
        for(int i = 0; i< rawFiducial.length; i++) {
            RawFiducial fiducial = rawFiducial[i];
            if (reefTags.contains(fiducial.id)) {
                if (bestTag == null || bestTag.distToCamera > fiducial.distToCamera) {
                    bestTag = fiducial;
                }
            }
        }
        if (bestTag == null) {
            return -1;
        } else {
            return bestTag.id;
        }
    }

    //calculates the pose of the robot relative to the reef tag based on whether or not we choose left or right side
    public Pose2d calculateReefPose(int tagID, boolean left) {
        Optional<Pose3d> pose3d = aprilTagFieldLayout.getTagPose(tagID); //gets the pose of the april tag in 3d space
        if (pose3d.isEmpty()) return null;

        if (left) {
            return pose3d.get().toPose2d().transformBy(leftReefTransform);
        } else {
            return pose3d.get().toPose2d().transformBy(rightReefTransform);
        }
    }

    public Pose2d calculateProcessorPose(boolean blue) {
        int tagID;
        if (blue) {
            tagID = blueProcessorTag;
        } else {
            tagID = redProcessorTag;
        }

        Optional<Pose3d> pose3d = aprilTagFieldLayout.getTagPose(tagID);
        if (pose3d.isEmpty()) return null;

        return pose3d.get().toPose2d().transformBy(processorTransform);
    }

    /**
     * Returns a scoring pose for the reef based on the last reef tag seen and the alliance color
     * @param left whether to return the left branch or the right branch scoring pose
     */
    public Pose2d getReefPose(boolean left) {
        if (left) {
            return leftReefHashMap.get(lastReefID);
        } else {
            return rightReefHashMap.get(lastReefID);
        }
    }

    public Pose2d getProcessorPose(boolean blue) {
        if (blue) {
            return blueProcessorPose;
        } else {
            return redProcessorPose;
        }
    }
    
    /**
     * Returns the closest reef scoring pose to the robot (accounting for alliance), or null
     */
    @NotLogged
    public Pose2d getClosestReefPose() {
        return closestReefPoseValid ? closestReefPose : null;
    }

    public int getReefTagFromPose(Pose2d pose) {
        for (Entry<Integer, Pose2d> entry : leftReefHashMap.entrySet()) {
            if (entry.getValue().equals(pose)) {
                return entry.getKey();
            }
        }
        for (Entry<Integer, Pose2d> entry : rightReefHashMap.entrySet()) {
            if (entry.getValue().equals(pose)) {
                return entry.getKey();
            }
        }
        return -1;
    }

    /**
     * Returns the position of the closest detected coral, or null
     */
    @NotLogged
    public Pose2d getCoralPose() {
        return coralPoseValid ? coralPose : null;
    }

    public Pose3d getCoralPose3D() {
        if (!coralPoseValid) return Pose3d.kZero;

        // TODO: recalculating this every loop is expensive
        return new Pose3d(coralPose);
    }

    public PoseEstimate validatePoseEstimate(PoseEstimate poseEstimate, double deltaSeconds) {
        if (poseEstimate == null) return null;

        if (megatag2Enabled) {
            if (poseEstimate.tagCount == 0) return null;
            if (Math.abs(RobotContainer.instance.drivetrain.getPigeon2().getRate()) > 720) return null;
        } else {
            double tagMin = 1;
            double tagMax = 99;
            double maxDist = poseEstimate.tagCount == 1 ? 3.7 : 6;
            double minArea = poseEstimate.tagCount == 1 ? 0.18 : 0.08;
            if (poseEstimate.tagCount > tagMax || poseEstimate.tagCount < tagMin) return null;
            if (poseEstimate.avgTagArea < minArea) return null;
            if (poseEstimate.avgTagDist > maxDist) return null;

            // Rejected if the pose estimate is too far from the last one
            // if (lastPoseEstimate != null && deltaSeconds <= 0.25) {
            //     double maxReasonableDistance = deltaSeconds * DrivetrainSubsystem.MAX_SPEED;
            //     Translation2d diff = poseEstimate.pose.getTranslation().minus(lastPoseEstimate.pose.getTranslation());
            //     if (!Helpers.withinTolerance(diff, maxReasonableDistance)) return null;
            // }
        }

        return poseEstimate;
    }

    @NotLogged
    public boolean reefVisible() {
        boolean isReefVisible = false;
        for (int i = 0; i < inputs.scoringFiducials.length; i++) {
            RawFiducial fiducial = inputs.scoringFiducials[i];
            if (redReefTags.contains(fiducial.id) || blueReefTags.contains(fiducial.id)) {
                isReefVisible = true;
            } else {
                isReefVisible = false;
            }

        }
        return isReefVisible;
    }
}
