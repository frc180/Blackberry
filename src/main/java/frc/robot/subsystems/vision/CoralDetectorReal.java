package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import com.spamrobotics.util.Helpers;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.LimelightHelpers.RawDetection;

@Logged
public class CoralDetectorReal implements CoralDetector {

    private final InterpolatingDoubleTreeMap distanceMap, reverseDistanceMap;
    private final List<RawDetection> sortedDetections;
    private final List<RawDetection> algaeDetections;
    private final Comparator<RawDetection> detectionTYComparator;
    private final Comparator<RawDetection> detectionTXYComparator;
    private final Comparator<RawDetection> detectionTXYWeightedComparator;
    private final boolean simulation;

    private static final double MAX_TY = 24.85;
    private static final double MAX_TX = 29.8;

    // How close to the robot a detected coral has to be to be considered "close" (i.e. intakeable)
    private final static double CLOSE_CORAL_DISTANCE = 0.6; // was 0.8 at SoFlo/Houston, 0.6 meters at Orlando
    private final static double CLOSE_CORAL_TX = 10;
    // How close two detected coral have to be to each other to be considered the same/close enough 
    // to allow switching without a timeout
    private final static double SIMILAR_CORAL_THRESHOLD = 0.75;

    private static final double ALGAE_AVOID_THRESHOLD_DEGREES = 2; // 4.5;

    private double lastDetectionTime = 0;
    private double lastDetectionDistance = 0;
    private double lastDetectionTX = 0;
    private double lastDetectionTY = 0;
    private double lastDetectionWidth = 0;
    private double lastDetectionHeight = 0;
    private double lastDetectionRatio = 0;
    private double lastDetectionCloseness = 0;
    private double lastDetectionTXYWeighted = 0;
    private Pose2d lastDetection = null;

    // Additional flags for viewing in logs
    private boolean newCoralValue = false;
    private boolean returningCloseDetection = false;
    private boolean rejectionAlgae = false;
    private boolean rejectionOutsideField = false;

    public CoralDetectorReal(boolean simulation) {
        this.simulation = simulation;
        distanceMap = new InterpolatingDoubleTreeMap();
        reverseDistanceMap = new InterpolatingDoubleTreeMap();
        addDistance(-22.2, 18.5);
        addDistance(-14.79, 24);
        addDistance(-6.05, 32);
        addDistance(5.44, 48);
        addDistance(14.89, 72);
        addDistance(20.82, 100);
        addDistance(24.7, 132);

        sortedDetections = new ArrayList<>();
        algaeDetections = new ArrayList<>();
        detectionTYComparator = (a, b) -> Double.compare(a.tync, b.tync);
        detectionTXYComparator = (a, b) -> Double.compare(tXYCombined(a), tXYCombined(b));
        detectionTXYWeightedComparator = (a, b) -> Double.compare(tXYWeighted(a), tXYWeighted(b));
    }

    private void addDistance(double ty, double inches) {
        double meters = Inches.of(inches).in(Meters);
        distanceMap.put(ty, meters);
        if (simulation) reverseDistanceMap.put(meters, ty);
    }

    @Override
    public Pose2d getCoralPose(Pose2d robotPose, RawDetection[] detections) {
        if (simulation) detections = getSimulatedRawDetections(robotPose);
        newCoralValue = false;
        returningCloseDetection = false;
        rejectionAlgae = false;
        rejectionOutsideField = false;
        sortedDetections.clear();
        algaeDetections.clear();
        
        Pose2d recentLastDetection = getRecentLastDetection();
        if (robotPose == null || detections == null || detections.length == 0) {
            return recentLastDetection;
        }

        boolean auto = RobotState.isAutonomous();

        // If the last detected coral was very close to the robot, wait a bit in case
        // we're trying to intake it
        if (recentLastDetection != null && auto && lastDetectionClose()) {
            returningCloseDetection = true;
            return recentLastDetection;
        }

        for (RawDetection detection : detections) {
            if (detection.classId == 1) {
                sortedDetections.add(detection);
            } else {
                algaeDetections.add(detection);
            }
        }
        if (auto) {
            // sortedDetections.sort(detectionTYComparator); // Champs
            sortedDetections.sort(detectionTXYWeightedComparator); // Experimental
        } else {
            sortedDetections.sort(detectionTXYComparator);
        }

        Pose2d basePose = getCameraPose(robotPose);

        for (RawDetection detection : sortedDetections) {
            double degrees = detection.txnc;

            // EXPERIMENT: Disable this entirely, or come up with a different way of dodging lollip coral (looking at width/height ratio?)
            if (false && auto) {
                // Skip any coral that are close to an algae on the X axis - these are likely lollipops
                for (RawDetection algae : algaeDetections) {
                    if (Math.abs(degrees - algae.txnc) < ALGAE_AVOID_THRESHOLD_DEGREES) {
                        rejectionAlgae = true;
                        break;
                    }
                }
                if (rejectionAlgae) continue;
            }

            double distanceMeters = distanceMap.get(detection.tync);
            double radians = Units.degreesToRadians(degrees);
            double yComponent = distanceMeters * Math.tan(radians);
            Transform2d coralTransform = new Transform2d(distanceMeters, -yComponent, Rotation2d.kZero);
            Pose2d coralPose = basePose.transformBy(coralTransform);

            if (!CoralDetector.isValid(coralPose)) {
                rejectionOutsideField = true;
                continue;
            }

            double robotDist = coralPose.getTranslation().getDistance(robotPose.getTranslation());
            // if (recentLastDetection != null && !auto) {
            //     double distDiff = coralPose.getTranslation().getDistance(recentLastDetection.getTranslation());
            //     if (distDiff > SIMILAR_CORAL_THRESHOLD) continue;
            // }

            lastDetection = coralPose;
            lastDetectionTime = Timer.getFPGATimestamp();
            lastDetectionDistance = robotDist;
            lastDetectionTX = detection.txnc;
            lastDetectionTY = detection.tync;
            lastDetectionWidth = width(detection);
            lastDetectionHeight = height(detection);
            lastDetectionRatio = lastDetectionWidth / lastDetectionHeight;
            lastDetectionCloseness = closeness(detection);
            lastDetectionTXYWeighted = tXYWeighted(detection);
            newCoralValue = true;
            return coralPose;
        }

        // If we didn't find any coral, return the last detection if it was very recent
        return recentLastDetection;
    }

    @Override
    public void reset() {
        lastDetection = null;
        lastDetectionDistance = 0;
        lastDetectionTime = 0;
    }

    private Pose2d getRecentLastDetection() {
        boolean lastClose = lastDetectionClose();
        boolean auto = RobotState.isAutonomous();

        // Champs
        // double timeoutSeconds = lastClose ? 3 : 0.5;
        // if (auto && lastClose) timeoutSeconds = 1;
        double timeoutSeconds = 0.5;
        if (Timer.getFPGATimestamp() - lastDetectionTime < timeoutSeconds) {
            return lastDetection;
        }
        return null;
    }

    public boolean lastDetectionClose() {
        if (lastDetection == null) return false;

        return lastDetectionDistance < CLOSE_CORAL_DISTANCE;// && Math.abs(lastDetectionTX) < CLOSE_CORAL_TX;
    }

    private double tXYCombined(RawDetection detection) {
        return detection.tync + Math.abs(detection.txnc * 0.75); // Used in teleop at champs
        // return detection.tync + Math.abs(detection.txnc * 0.1); // was 0.33 at orlando
    }

    private double tXYWeighted(RawDetection detection) {
        final double TX_BEGIN_CLOSE = 0;
        final double TX_RELATIVE_WEIGHT = 1;
        final double TX_MAX_WEIGHT = 1;

        double closeness = closeness(detection);
        double txWeight = 0;
        if (closeness > TX_BEGIN_CLOSE) {
            double rescaledCloseness = (closeness - TX_BEGIN_CLOSE) / (1 - TX_BEGIN_CLOSE);
            txWeight = Math.max(TX_RELATIVE_WEIGHT * rescaledCloseness, TX_MAX_WEIGHT);
        }
        return detection.tync + (Math.abs(detection.txnc) * txWeight);
    }

    private double width(RawDetection detection) {
        return Math.abs(detection.corner0_X - detection.corner1_X);
    }

    private double height(RawDetection detection) {
        return Math.abs(detection.corner0_Y - detection.corner2_Y);
    }

    /**
     * Returns a value from 0 to 1, where 0 is the furthest and 1 is the closest
     */
    private double closeness(RawDetection detection) {
        double value = (detection.tync + MAX_TY) / (MAX_TY * 2);
        if (value < 0) {
            value = 0;
            System.out.println("CoralDetectorReal: Closeness value < 0! " + value);
        } else if (value > 1) {
            value = 1;
            System.out.println("CoralDetectorReal: Closeness value > 1! " + value);
        }
        return 1 - value;
    }

    private Pose2d getCameraPose(Pose2d robotPose) {
        return robotPose.transformBy(VisionSubsystem.ROBOT_TO_INTAKE_CAMERA_2D);
    }
    // ========================== Simulation Methods ==========================

    final double fovDegrees = 82;
    final double detectionDistance = 5;

    private RawDetection[] getSimulatedRawDetections(Pose2d robotPose) {
        final List<RawDetection> detections = new ArrayList<>();
        final Pose2d cameraPose = getCameraPose(robotPose);
        final Pose3d[] corals = SimulatedArena.getInstance().getGamePiecesArrayByType("Coral");

        for (int i = 0; i < corals.length; i++) {
            Pose2d coral = corals[i].toPose2d();
            if (Helpers.poseWithinPOV(robotPose, coral, fovDegrees, detectionDistance)) {
                Pose2d relative = coral.relativeTo(cameraPose); 
                RawDetection detection = new RawDetection();
                detection.classId = 1;
                detection.tync = reverseDistanceMap.get(relative.getX());
                detection.txnc = -Units.radiansToDegrees(Math.atan2(relative.getY(), relative.getX()));
                detections.add(detection);
            }
        }

        RawDetection[] detectionArray = new RawDetection[detections.size()];
        for (int i = 0; i < detections.size(); i++) {
            detectionArray[i] = detections.get(i);
        }
        return detectionArray;
    }
}
