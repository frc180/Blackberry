package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.util.ReefProximity;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.LimelightHelpers.RawFiducial;

@Logged
public class VisionSubsystem extends SubsystemBase {

    /**
     * The source of a pose estimate, used to determine the standard deviation of the pose estimate
     * (i.e. how much we trust the pose estimate). The lower the number, the more we trust the pose estimate,
     * and the more it'll affect the robot's position.
     */
    enum PoseEstimateSource {
        SCORING_CAMERA(0.05),
        FRONT_CAMERA(0.3),
        BACK_CAMERA(0.3),
        NONE(99);

        private Matrix<N3, N1> stdDev;

        PoseEstimateSource(double dev) {
            this(dev, dev, dev);
        }

        PoseEstimateSource(double xDev, double yDev, double thetaDev) {
            stdDev = MatBuilder.fill(Nat.N3(), Nat.N1(), xDev, yDev, thetaDev);
        }
    } 

    private static final List<Integer> redTags = List.of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11);
    private static final List<Integer> blueTags = List.of(12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22);
    public static final List<Integer> redReefTags = List.of(6,7,8,9,10,11);
    public static final List<Integer> blueReefTags = List.of(17, 18, 19, 20, 21, 22);

    // x and y may need to be flipped
    public static final Transform3d ROBOT_TO_SCORING_CAMERA = new Transform3d(
        Inches.of(9.757).in(Meters),
        Inches.of(-7.2).in(Meters),
        Inches.of(31.296).in(Meters),
        new Rotation3d(0, Units.degreesToRadians(50), 0)
    );

    // TODO: set real values
    public static final Transform3d ROBOT_TO_FRONT_CAMERA = new Transform3d();
    
    // TODO: set real values
    public static final Transform2d ROBOT_TO_INTAKE_CAMERA = new Transform2d(0.1, 0, Rotation2d.fromDegrees(0));
    public static final Transform2d INTAKE_CAMERA_TO_ROBOT = ROBOT_TO_INTAKE_CAMERA.inverse();

    private static final int RED_PROCESSOR_TAG = 3;
    private static final int BLUE_PROCESSOR_TAG = 16;

    private static final double BAD_CAMERA_TEMP = 60;

    private final VisionIO io;
    private final VisionIOInputs inputs;

    private final ReefProximity reefProximity;
    private final CoralDetector coralDetector;
    private final SingleTagSolver singleTagSolver = new SingleTagSolver();

    private final Transform2d leftReefTransform = new Transform2d(0.55, -0.15, Rotation2d.fromDegrees(180));
    private final Transform2d rightReefTransform = new Transform2d(0.55, 0.15, Rotation2d.fromDegrees(180));
    private final Transform2d processorTransform = new Transform2d(0.55, 0.0, Rotation2d.fromDegrees(90));

    private final Pose2d redProcessorPose;
    private final Pose2d blueProcessorPose;

    public final HashMap<Integer, Pose2d> leftReefHashMap = new HashMap<>();
    public final HashMap<Integer, Pose2d> rightReefHashMap = new HashMap<>();

    public AprilTagFieldLayout aprilTagFieldLayout;
    public final Trigger poseEstimateDiffLow;
    @NotLogged
    public final Trigger scoringCameraConnected;


    private boolean canSeeReef = false;
    public int bestReefID = -1;
    public int lastReefID = -1;

    private Pose2d coralPose = Pose2d.kZero;
    private boolean coralPoseValid = false;

    private boolean closestReefPoseValid = false;
    private Pose2d closestReefPose = Pose2d.kZero;

    private Pose2d singleTagPose = Pose2d.kZero;

    final boolean megatag2Enabled = false;

    private PoseEstimate poseEstimate = null;
    private PoseEstimateSource poseEstimateSource = PoseEstimateSource.NONE;
    private Pose3d scoringCameraPosition = Pose3d.kZero;
    private double poseEstimateDiffX, poseEstimateDiffY, poseEstimateDiffTheta;
    
    private Alert scoringCameraDisconnectedAlert = new Alert("Scoring Camera disconnected!", AlertType.kError);
    private Alert frontCameraDisconnectedAlert = new Alert("Front Camera disconnected!", AlertType.kError);
    private Alert backCameraDisconnectedAlert = new Alert("Back Camera disconnected!", AlertType.kError);

    private Alert scoringCameraTempAlert = new Alert("", AlertType.kWarning);

    @SuppressWarnings("unused")
    public VisionSubsystem() {
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
        } catch (Exception e) {
            aprilTagFieldLayout = null;
        }

        inputs = new VisionIOInputs();
        if (Robot.isReal()) {
            io = new VisionIOLimelight();
        } else {
            io = new VisionIOPhoton(aprilTagFieldLayout);
        }

        // Pre-calculate reef poses for all reef tags
        for (int i : redReefTags) {
            leftReefHashMap.put(i, calculateReefPose(i, true));
            rightReefHashMap.put(i, calculateReefPose(i, false));
        }
        for (int i : blueReefTags) {
            leftReefHashMap.put(i, calculateReefPose(i, true));
            rightReefHashMap.put(i, calculateReefPose(i, false));
        }
        
        // Pre-calculate processor poses
        redProcessorPose = calculateProcessorPose(false);
        blueProcessorPose = calculateProcessorPose(true);

        reefProximity = new ReefProximity(leftReefHashMap, rightReefHashMap);
        coralDetector = Robot.isReal() ? new CoralDetectorReal() : new CoralDetectorSim(4.0, true);

        double diffMeters = Inches.of(1.5).in(Meters);
        poseEstimateDiffLow = new Trigger(() -> {
            return Math.abs(poseEstimateDiffX) <= diffMeters && 
                   Math.abs(poseEstimateDiffY) <= diffMeters && 
                   Math.abs(poseEstimateDiffTheta) < 5;
        });
        scoringCameraConnected = new Trigger(() -> inputs.scoringCameraConnected);
    }

    @Override
    public void periodic() {
        io.update(inputs);

        scoringCameraDisconnectedAlert.set(!inputs.scoringCameraConnected);
        frontCameraDisconnectedAlert.set(!inputs.frontCameraConnected);
        backCameraDisconnectedAlert.set(!inputs.backCameraConnected);

        cameraTemperatureAlert(scoringCameraTempAlert, "Scoring", Math.max(inputs.scoringTemp, inputs.scoringCPUTemp));

        // If the scoring camera is connected, use its pose estimate
        if (inputs.scoringCameraConnected) {
            poseEstimate = validatePoseEstimate(inputs.scoringPoseEstimate, 0);
            poseEstimateSource = PoseEstimateSource.SCORING_CAMERA;
        }

        // If poseEstimate is null, then try the front camera pose estimate
        if (poseEstimate == null && inputs.frontCameraConnected) {
            poseEstimate = validatePoseEstimate(inputs.frontPoseEstimate, 0);
            poseEstimateSource = PoseEstimateSource.FRONT_CAMERA;
        }
     
        Pose2d robotPose = null;
        if (poseEstimate != null) {
            RobotContainer.instance.drivetrain.addVisionMeasurement(
                poseEstimate.pose,
                Utils.fpgaToCurrentTime(poseEstimate.timestampSeconds),
                poseEstimateSource.stdDev
            );
            // Calculate the difference between the updated robot pose and the scoring pose estimate, to get an idea
            // of how closely we are tracking the robot's actual position
            robotPose = RobotContainer.instance.drivetrain.getPose();
            poseEstimateDiffX = robotPose.getX() - poseEstimate.pose.getX();
            poseEstimateDiffY = robotPose.getY() - poseEstimate.pose.getY();
            poseEstimateDiffTheta = robotPose.getRotation().getDegrees() - poseEstimate.pose.getRotation().getDegrees();
        } else {
            poseEstimateSource = PoseEstimateSource.NONE;
        }

        if (robotPose == null) robotPose = RobotContainer.instance.drivetrain.getPose();

        // calculate scoring camera in 3D space, for viewing in AdvantageScope
        Pose2d cameraPosebase = Robot.isReal() ? robotPose : RobotContainer.instance.drivetrain.getSimPose();
        scoringCameraPosition = new Pose3d(cameraPosebase).transformBy(ROBOT_TO_SCORING_CAMERA);

        //check if robot can see the reef
        canSeeReef = reefVisible();

        Entry<Integer, Pose2d> closestTagAndPose = reefProximity.closestReefPose(robotPose, Robot.isBlue());
        if (closestTagAndPose == null) {
            closestReefPose = Pose2d.kZero;
            closestReefPoseValid = false;
        } else {
            closestReefPose = closestTagAndPose.getValue();
            closestReefPoseValid = true;
        }

        Pose2d latencyCompensatedRobotPose;
        if (Robot.isReal()) {
            latencyCompensatedRobotPose = RobotContainer.instance.drivetrain.getBufferPose(inputs.backTimestamp);
        } else {
            latencyCompensatedRobotPose = robotPose;
        }
        coralPose = coralDetector.getCoralPose(latencyCompensatedRobotPose, inputs.backDetections);
        if (coralPose == null) {
            coralPose = Pose2d.kZero;
            coralPoseValid = false;
        } else {
            coralPoseValid = true;
        }

        // For now, we're just going to use the closest reef tag to the robot
        if (false && inputs.scoringCameraConnected) {
            bestReefID = getReefTag(inputs.scoringFiducials);
        } else {
            // Simulate the vision system by selecting the closest reef tag to the robot position
            if (closestTagAndPose == null) {
                bestReefID = -1;
            } else {
                bestReefID = closestTagAndPose.getKey();
            }
        }

        if (bestReefID != -1) lastReefID = bestReefID;

        if (inputs.scoringFiducials.length > 0) {
            // TODO: If this seems more accurate than global pose, we need to specifically pass in the scoring fiducial
            // that corresponds to the tag we're actively targeting, not just the first or closest one
            singleTagPose = singleTagSolver.getPose(inputs.scoringTimestamp, inputs.scoringFiducials[0], ROBOT_TO_SCORING_CAMERA);
        }
    }

    
    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public int getReefTag(RawFiducial[] rawFiducial) {
        RawFiducial bestTag = null;
        List<Integer> reefTags = Robot.isBlue() ? blueReefTags : redReefTags;
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

    /**
     * Generates the scoring pose of the robot relative to a reef AprilTag. This is used to pre-calculate and store all
     * positions to prevent duplicate object creation. To access these pre-calculated poses, use {@link #getReefPose(int, boolean)}.
     */
    private Pose2d calculateReefPose(int tagID, boolean left) {
        Optional<Pose3d> pose3d = aprilTagFieldLayout.getTagPose(tagID); //gets the pose of the april tag in 3d space
        if (pose3d.isEmpty()) return null;

        return pose3d.get().toPose2d().transformBy(left ? leftReefTransform : rightReefTransform);
    }

    /**
     * Generates the scoring pose of the robot relative to the processor AprilTag. This is used to pre-calculate and store all 
     * positions to prevent duplicate object creation. To access these pre-calculated poses, use {@link #getProcessorPose(boolean)}.
     */
    private Pose2d calculateProcessorPose(boolean blue) {
        int tagID = blue ? BLUE_PROCESSOR_TAG : RED_PROCESSOR_TAG;

        Optional<Pose3d> pose3d = aprilTagFieldLayout.getTagPose(tagID);
        if (pose3d.isEmpty()) return null;

        return pose3d.get().toPose2d().transformBy(processorTransform);
    }

    /**
     * Returns the scoring pose of the robot relative to the reef AprilTag that was last seen.
     * @param left whether to return the left branch or the right branch scoring pose
     */
    public Pose2d getReefPose(boolean left) {
        return getReefPose(lastReefID, left);
    }

    /**
     * Returns a scoring pose of the robot relative to a reef AprilTag.
     * @param tagID the ID of the reef AprilTag
     * @param left whether to return the left branch or the right branch scoring pose
     */
    public Pose2d getReefPose(int tagID, boolean left) {
        if (left) {
            return leftReefHashMap.get(tagID);
        } else {
            return rightReefHashMap.get(tagID);
        }
    }

    /**
     * Returns the scoring pose of the robot relative to the processor.
     * @param blue whether to return the blue processor pose or the red processor pose
     */
    public Pose2d getProcessorPose(boolean blue) {
        return blue ? blueProcessorPose : redProcessorPose;
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

    private void cameraTemperatureAlert(Alert alert, String cameraName, double temperature) {
        if (temperature >= BAD_CAMERA_TEMP) {
            int roundedtemp = (int) Math.round(temperature);
            alert.setText(cameraName + " Camera temp high (" + roundedtemp + "°F)");
            alert.set(true);
        } else {
            alert.set(false);
        }
    }
}
