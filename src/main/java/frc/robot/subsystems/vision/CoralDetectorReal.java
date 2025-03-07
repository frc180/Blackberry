package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.LimelightHelpers.RawDetection;

public class CoralDetectorReal implements CoralDetector {

    private final InterpolatingDoubleTreeMap distanceMap;
    private final List<RawDetection> sortedDetections;
    private final Comparator<RawDetection> detectionTYComparator;

    public CoralDetectorReal() {
        // TODO: confirm what min and max ty are and what the distance is for each
        distanceMap = new InterpolatingDoubleTreeMap();
        distanceMap.put(-22.2, Inches.of(18.5).in(Meters));
        distanceMap.put(-14.79, Inches.of(24).in(Meters));
        distanceMap.put(-6.05, Inches.of(32).in(Meters));
        distanceMap.put(5.44, Inches.of(48).in(Meters));
        distanceMap.put(14.89, Inches.of(72).in(Meters));
        distanceMap.put(20.82, Inches.of(100).in(Meters));
        distanceMap.put(24.7, Inches.of(132).in(Meters));


        sortedDetections = new ArrayList<>();
        detectionTYComparator = (a, b) -> Double.compare(a.tync, b.tync);
    }

    @Override
    public Pose2d getCoralPose(Pose2d robotPose, RawDetection[] detections) {
        if (robotPose == null || detections == null || detections.length == 0) {
            return null;
        }

        sortedDetections.clear();
        for (RawDetection detection : detections) {
            // Skip non-coral detections
            if (detection.classId != 1) continue;

            sortedDetections.add(detection);
        }
        sortedDetections.sort(detectionTYComparator);

        // Pose2d basePose = robotPose;
        Pose2d basePose = robotPose.transformBy(VisionSubsystem.ROBOT_TO_INTAKE_CAMERA_2D);

        for (RawDetection detection : sortedDetections) {
            double distanceMeters = distanceMap.get(detection.tync);
            double degrees = detection.txnc;
            double radians = Units.degreesToRadians(degrees);

            SmartDashboard.putBoolean("Coral Present", true);
            SmartDashboard.putNumber("Coral TY", detection.tync);
            SmartDashboard.putNumber("Coral TX", detection.txnc);
            // SmartDashboard.putNumber("Coral Distance", Meters.of(distanceMeters).in(Inches));

            double yComponent = (distanceMeters)*Math.tan(radians);
            Transform2d coralTransform = new Transform2d(distanceMeters, -yComponent, Rotation2d.kZero);
            Pose2d coralPose = basePose.transformBy(coralTransform);

            // TODO: reject coralPoses that are outside the field or (perhaps) where the stacked coral are
            return coralPose;
        }
        SmartDashboard.putBoolean("Coral Present", false);
        return null;
    }
}
