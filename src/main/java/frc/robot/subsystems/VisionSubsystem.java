package frc.robot.subsystems;

import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.Map.Entry;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.ReefProximity;
import frc.robot.util.LimelightHelpers.RawFiducial;

@Logged
public class VisionSubsystem extends SubsystemBase {

    private static final String SCORING_LIMELIGHT = "limelight";
    public AprilTagFieldLayout aprilTagFieldLayout;

    private boolean canSeeReef = false;
    public int bestReefID = -1;

    private final RawFiducial[] emptyFiducials = new RawFiducial[0];
    public RawFiducial[] rawFiducials = emptyFiducials;

    public int lastReefID;

    int[] fiducialArray = new int[0];

    private final ReefProximity reefProximity;

    private List<Integer> redTags = List.of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11);
    private List<Integer> blueTags = List.of(12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22);

    public static final List<Integer> redReefTags = List.of(6,7,8,9,10,11);
    public static final List<Integer> blueReefTags = List.of(17, 18, 19, 20, 21, 22);

    private final Transform2d leftReefTransform = new Transform2d(0.5, -0.25, new Rotation2d());
    private final Transform2d rightReefTransform = new Transform2d(0.5, 0.25, new Rotation2d());

    private final HashMap<Integer, Pose2d> leftReefHashMap = new HashMap<>();
    private final HashMap<Integer, Pose2d> rightReefHashMap = new HashMap<>();

    private Pose2d exampleLeft;
    private Pose2d exampleRight;

    private boolean closestReefPoseValid = false;
    private Pose2d closestReefPose = Pose2d.kZero;
    
    public VisionSubsystem() {

        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
        } catch (Exception e) {
            aprilTagFieldLayout = null;
        }

        //red reef tags
        for (int i = 6; i <=11; i++) {
            leftReefHashMap.put(i, calculateReefPose(i, true));
            rightReefHashMap.put(i, calculateReefPose(i, false));
        }
        //left reef tags
        for (int i = 17; i <= 22; i++) {
            leftReefHashMap.put(i, calculateReefPose(i, true));
            rightReefHashMap.put(i, calculateReefPose(i, false));
        }

        exampleLeft = leftReefHashMap.get(18);
        exampleRight = rightReefHashMap.get(18);

        reefProximity = new ReefProximity(leftReefHashMap, rightReefHashMap);
    }

    @Override
    public void periodic() {
        Pose2d robotPose = RobotContainer.instance.drivetrain.getPose();
        Entry<Integer, Pose2d> closestTagAndPose = reefProximity.closestReefPose(robotPose, Robot.isBlue());
        if (closestTagAndPose == null) {
            closestReefPose = Pose2d.kZero;
            closestReefPoseValid = false;
        } else {
            closestReefPose = closestTagAndPose.getValue();
            closestReefPoseValid = true;
        }

        if (Robot.isReal()) {
            rawFiducials = LimelightHelpers.getRawFiducials(SCORING_LIMELIGHT);
            fiducialArray = new int[rawFiducials.length];
            for(int i = 0; i < rawFiducials.length; i++) {
                fiducialArray[i] = rawFiducials[i].id;
            }

            bestReefID = getReefTag(rawFiducials);
            System.out.println(bestReefID);
        } else {
            // Simulate the vision system by selecting the closest reef tag to the robot position
            if (closestTagAndPose == null) {
                bestReefID = -1;
            } else {
                bestReefID = closestTagAndPose.getKey();
            }
        }
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

    /**
     * Returns a scoring pose for the reef based on the last reef tag seen and the alliance color
     * @param left whether to return the left branch or the right branch scoring pose
     */
    public Pose2d getReefPose(boolean left) {
        if (left) {
            return leftReefHashMap.get(bestReefID);
        } else {
            return rightReefHashMap.get(bestReefID);
        }
    }
    
    /**
     * Returns the closest reef scoring pose to the robot (accounting for alliance), or null
     */
    public Pose2d getClosestReefPose() {
        return closestReefPoseValid ? closestReefPose : null;
    }
}
