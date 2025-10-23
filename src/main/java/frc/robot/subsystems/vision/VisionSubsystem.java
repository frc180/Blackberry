package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.util.FlippingUtil;
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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.util.ReefProximity;
import frc.robot.util.LimelightHelpers.PoseEstimate;

@Logged
public class VisionSubsystem extends SubsystemBase {

    /**
     * The source of a pose estimate, used to determine the standard deviation of the pose estimate
     * (i.e. how much we trust the pose estimate). The lower the number, the more we trust the pose estimate,
     * and the more it'll affect the robot's position.
     */
    enum PoseEstimateSource {
        SCORING_CAMERA(0.025), // was 0.05
        FRONT_CAMERA(0.15), // was .3
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
    public static final List<Integer> allReefTags = new ArrayList<>();

    /**
     * original Limelight 3G position (going to be invalid)
     * 5/8in left
     * 14 5/8 forward
     * 11 3/8 high
     */

    public static final Transform3d ROBOT_TO_FRONT_CAMERA = new Transform3d(
        Inches.of(14 + 5.0/8.0).in(Meters), // forward
        Inches.of(5.0/8.0).in(Meters), // left?
        Inches.of(11 + 3.0/8.0).in(Meters),
        new Rotation3d(0, Units.degreesToRadians(0), 0) // no tilt
    );


    private final VisionIO io;
    private final VisionIOInputs inputs;

    private final ReefProximity reefProximity;

    private final Distance reefBackDistance = Meters.of(0.55).plus(Inches.of(0.5 - 1));
    private final Distance reefSideDistance = Inches.of(2);

    private final Transform2d leftReefTransform = new Transform2d(reefBackDistance.in(Meters), -reefSideDistance.in(Meters), Rotation2d.k180deg);
    private final Transform2d rightReefTransform = new Transform2d(reefBackDistance.in(Meters), reefSideDistance.in(Meters), Rotation2d.k180deg);

    public final HashMap<Integer, Pose2d> tagPoses2d = new HashMap<>();
    public final HashMap<Integer, Pose2d> leftReefHashMap = new HashMap<>();
    public final HashMap<Integer, Pose2d> rightReefHashMap = new HashMap<>();

    public int bestReefID = -1;
    public int lastReefID = -1;

    private Pose2d coralPose = Pose2d.kZero;
    private boolean coralPoseValid = false;
    private Pose2d coralPickupPose = null;

    private boolean closestReefPoseValid = false;
    private Pose2d closestReefPose = Pose2d.kZero;

    private Pose2d singleTagPose = Pose2d.kZero;

    private PoseEstimate poseEstimate = null;
    private PoseEstimateSource poseEstimateSource = PoseEstimateSource.NONE;
    private boolean allowPoseEstimates = true;
    private Pose3d frontCameraPosition = Pose3d.kZero;
    private double poseEstimateDiffX, poseEstimateDiffY, poseEstimateDiffTheta;
    private double lastPoseEstimateTime = 0;
    
    private Alert frontCameraDisconnectedAlert = new Alert("Front Camera disconnected!", AlertType.kError);

    public AprilTagFieldLayout aprilTagFieldLayout;

    @SuppressWarnings("unused")
    public VisionSubsystem() {
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeAndyMark.m_resourceFile);
        } catch (Exception e) {
            aprilTagFieldLayout = null;
        }

        inputs = new VisionIOInputs();
        if (Robot.isReal()) {
            io = new VisionIOLimelight();
        } else {
            io = new VisionIOPhoton(aprilTagFieldLayout);
        }

        allReefTags.addAll(redReefTags);
        allReefTags.addAll(blueReefTags);

        for (var allianceTags : List.of(redTags, blueTags)) {
            for (int tagID : allianceTags) {
                Optional<Pose3d> pose3d = aprilTagFieldLayout.getTagPose(tagID);
                if (pose3d.isPresent()) {
                    tagPoses2d.put(tagID, pose3d.get().toPose2d());
                }
            }
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

        reefProximity = new ReefProximity(leftReefHashMap, rightReefHashMap);
    }

    boolean wasEnabled = false;

    Pose2d futureRobotPose;

    @Override
    public void periodic() {
        io.update(inputs);

        frontCameraDisconnectedAlert.set(!inputs.frontCameraConnected);

        // use the front camera's pose estimate
        if (inputs.frontCameraConnected) {
            poseEstimateSource = PoseEstimateSource.FRONT_CAMERA;
            poseEstimate = validatePoseEstimate(inputs.frontPoseEstimate);
        }
     
        Pose2d robotPose = null;
        if (poseEstimate != null) {
            RobotContainer.instance.drivetrain.addVisionMeasurement(
                poseEstimate.pose,
                Utils.fpgaToCurrentTime(poseEstimate.timestampSeconds),
                poseEstimateSource.stdDev
            );
            lastPoseEstimateTime = Timer.getFPGATimestamp();
        } else {
            poseEstimateSource = PoseEstimateSource.NONE;
        }

        if (robotPose == null) robotPose = RobotContainer.instance.drivetrain.getPose();

        // calculate scoring camera in 3D space, for previewing in AdvantageScope
        if (Robot.isSimulation()) {
            Pose3d robotPose3d = new Pose3d(RobotContainer.instance.drivetrain.getSimPose());
            frontCameraPosition = robotPose3d.transformBy(ROBOT_TO_FRONT_CAMERA);
        }

        ChassisSpeeds speeds = RobotContainer.instance.drivetrain.getCachedState().Speeds;
        futureRobotPose = robotPose.plus(new Transform2d(speeds.vxMetersPerSecond * 0.3, speeds.vyMetersPerSecond * 0.3, Rotation2d.kZero));

        // Allow targeting opponent's reef tags, which is needed for stealing algae
        // Entry<Integer, Pose2d> closestTagAndPose = reefProximity.closestReefPose(futureRobotPose, Robot.isBlue());
        Entry<Integer, Pose2d> closestTagAndPose = reefProximity.closestReefPose(futureRobotPose, allReefTags);
        if (closestTagAndPose == null) {
            closestReefPose = Pose2d.kZero;
            closestReefPoseValid = false;
        } else {
            closestReefPose = closestTagAndPose.getValue();
            closestReefPoseValid = true;
        }

        if (closestTagAndPose == null) {
            bestReefID = -1;
        } else {
            bestReefID = closestTagAndPose.getKey();
        }

        if (bestReefID != -1) lastReefID = bestReefID;
    }

    
    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    private Pose2d getReefTagPose(int tagID) {
        Pose2d pose = tagPoses2d.get(tagID);
        if (pose == null) return null;

        return pose;
    }

    /**
     * Generates the scoring pose of the robot relative to a reef AprilTag. This is used to pre-calculate and store all
     * positions to prevent duplicate object creation. To access these pre-calculated poses, use {@link #getReefPose(int, boolean)}.
     */
    private Pose2d calculateReefPose(int tagID, boolean left) {
        Pose2d pose = getReefTagPose(tagID);
        if (pose == null) return null;

        return pose.transformBy(left ? leftReefTransform : rightReefTransform);
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
     * Returns the closest reef scoring pose to the robot (accounting for alliance), or null
     */
    @NotLogged
    public Pose2d getClosestReefPose() {
        return closestReefPoseValid ? closestReefPose : null;
    }

    private final static double BARGE_BLUE_X = 7.28;
    private final static double BARGE_RED_X = FlippingUtil.fieldSizeX - BARGE_BLUE_X;

    public Pose2d getBargePose() {
        Pose2d robotPose = RobotContainer.instance.drivetrain.getPose();

        return new Pose2d(
            Robot.isBlue() ? BARGE_BLUE_X : BARGE_RED_X,
            robotPose.getY(),
            Robot.isBlue() ? Rotation2d.kZero : Rotation2d.k180deg
        );
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

    public boolean isFrontCameraConnected() {
        return inputs.frontCameraConnected;
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

        return new Pose3d(coralPose);
    }

    public PoseEstimate validatePoseEstimate(PoseEstimate poseEstimate) {
        if (poseEstimate == null) return null;

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

        return poseEstimate;
    }

    public PoseEstimate validateMT2PoseEstimate(PoseEstimate poseEstimate) {
        if (poseEstimate == null) return null;
        if (poseEstimate.tagCount == 0) return null;
        // if (Math.abs(RobotContainer.instance.drivetrain.getGyroscopeRate()) > 720) return null;

        return poseEstimate;
    }

    /**
     * Blocks pose estimates from updating the robot's position while this command is running.
     */
    public Command blockPoseEstimates() {
        return Commands.runEnd(
            () -> allowPoseEstimates = false,
            () -> allowPoseEstimates = true
        );
    }

    public void setAllowPoseEstimates(boolean allow) {
        allowPoseEstimates = allow;
    }
}
