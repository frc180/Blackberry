package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.LimelightHelpers.RawDetection;

@Logged
public class CoralDetectorReal implements CoralDetector {

    private final InterpolatingDoubleTreeMap distanceMap;
    private final List<RawDetection> sortedDetections;
    private final Comparator<RawDetection> detectionTYComparator;
    private final Comparator<RawDetection> detectionTXYComparator;
    private final MutDistance coralDistance;

    // How close to the robot a detected coral has to be to be considered "close" (i.e. intakeable)
    private final double CLOSE_CORAL_DISTANCE = 0.75;
    private final double CLOSE_CORAL_TX = 10;
    // How close two detected coral have to be to each other to be considered the same/close enough 
    // to allow switching without a timeout
    private final double SIMILAR_CORAL_THRESHOLD = 0.75;

    // Experimental
    private final double CORAL_FURTHER_THRESHOLD = 0.6;

    private double lastDetectionTime = 0;
    private double lastDetectionDistance = 0;
    private double lastDetectionTX = 0;
    private Pose2d lastDetection = null;

    public CoralDetectorReal() {
        distanceMap = new InterpolatingDoubleTreeMap();
        // distanceMap.put(-22.2, Inches.of(18.5).in(Meters));
        // distanceMap.put(-14.79, Inches.of(24).in(Meters));
        // distanceMap.put(-6.05, Inches.of(32).in(Meters));
        // distanceMap.put(5.44, Inches.of(48).in(Meters));
        // distanceMap.put(14.89, Inches.of(72).in(Meters));
        // distanceMap.put(20.82, Inches.of(100).in(Meters));
        // distanceMap.put(24.7, Inches.of(132).in(Meters));

        addDistance(-22.2, 18.5);
        addDistance(-14.79, 24);
        addDistance(-6.05, 32);
        addDistance(5.44, 48);
        addDistance(14.89, 72);
        addDistance(20.82, 100);
        addDistance(24.7, 132);

        sortedDetections = new ArrayList<>();
        detectionTYComparator = (a, b) -> Double.compare(a.tync, b.tync);
        detectionTXYComparator = (a, b) -> Double.compare(tXYCombined(a), tXYCombined(b));
        coralDistance = Meters.of(0).mutableCopy();
    }

    private void addDistance(double ty, double inches) {
        distanceMap.put(ty, Inches.of(inches).in(Meters));
    }

    @Override
    public Pose2d getCoralPose(Pose2d robotPose, RawDetection[] detections) {
        Pose2d recentLastDetection = getRecentLastDetection();
        if (robotPose == null || detections == null || detections.length == 0) {
            return recentLastDetection;
        }

        // If the last detected coral was very close to the robot, wait a bit in case
        // we're trying to intake it
        if (recentLastDetection != null && lastDetectionClose()) {
            return recentLastDetection;
        }

        sortedDetections.clear();
        for (RawDetection detection : detections) {
            // Skip non-coral detections
            if (detection.classId != 1) continue;

            sortedDetections.add(detection);
        }
        sortedDetections.sort(detectionTXYComparator);
        // sortedDetections.sort(detectionTYComparator);

        Pose2d basePose = robotPose.transformBy(VisionSubsystem.ROBOT_TO_INTAKE_CAMERA_2D);

        for (RawDetection detection : sortedDetections) {
            double distanceMeters = distanceMap.get(detection.tync);
            double degrees = detection.txnc;
            double radians = Units.degreesToRadians(degrees);

            SmartDashboard.putBoolean("Coral Present", true);
            SmartDashboard.putNumber("Coral TY", detection.tync);
            SmartDashboard.putNumber("Coral TX", detection.txnc);
            SmartDashboard.putNumber("Coral Distance", coralDistance.mut_replace(distanceMeters, Meters).in(Inches));
            SmartDashboard.putNumber("Coral Width", width(detection));
            SmartDashboard.putNumber("Coral Height", height(detection));
            SmartDashboard.putNumber("Coral Ratio", width(detection) / height(detection));

            double yComponent = distanceMeters * Math.tan(radians);
            Transform2d coralTransform = new Transform2d(distanceMeters, -yComponent, Rotation2d.kZero);
            Pose2d coralPose = basePose.transformBy(coralTransform);

            if (!CoralDetector.isValid(coralPose)) {
                continue;
            }

            double robotDist = coralPose.getTranslation().getDistance(robotPose.getTranslation());
            if (recentLastDetection != null && !RobotState.isAutonomous()) {
                // double distDiff = coralPose.getTranslation().getDistance(recentLastDetection.getTranslation());
                // if (distDiff > SIMILAR_CORAL_THRESHOLD) continue;
                if (robotDist > lastDetectionDistance + CORAL_FURTHER_THRESHOLD) continue;
            }

            lastDetectionTime = Timer.getFPGATimestamp();
            lastDetectionDistance = robotDist;
            lastDetectionTX = detection.txnc;
            lastDetection = coralPose;
            return coralPose;
        }

        SmartDashboard.putBoolean("Coral Present", false);
        // If we didn't find any coral, return the last detection if it was very recent
        return recentLastDetection;
    }

    @Override
    public void reset() {
        lastDetection = null;
        lastDetectionDistance = 0;
        lastDetectionTime = 0;
    }

    @NotLogged
    private Pose2d getRecentLastDetection() {
        boolean lastClose = lastDetectionClose();
        if (RobotState.isAutonomous() && lastClose) return lastDetection;

        double timeoutSeconds = lastClose ? 3 : 0.5;
        if (Timer.getFPGATimestamp() - lastDetectionTime < timeoutSeconds) {
            return lastDetection;
        }
        return null;
    }

    private boolean lastDetectionClose() {
        if (lastDetection == null) return false;

        return lastDetectionDistance < CLOSE_CORAL_DISTANCE;// && Math.abs(lastDetectionTX) < CLOSE_CORAL_TX;
    }

    private double tXYCombined(RawDetection detection) {
        return detection.tync + Math.abs(detection.txnc * 0.33);
    }

    private double width(RawDetection detection) {
        return Math.abs(detection.corner0_X - detection.corner1_X);
    }

    private double height(RawDetection detection) {
        return Math.abs(detection.corner0_Y - detection.corner2_Y);
    }
}
