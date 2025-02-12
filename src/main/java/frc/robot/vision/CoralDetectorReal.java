package frc.robot.vision;

import static edu.wpi.first.units.Units.*;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.LimelightHelpers.RawDetection;

public class CoralDetectorReal implements CoralDetector {

    private final InterpolatingDoubleTreeMap distanceMap;
    private final List<RawDetection> sortedDetections;
    private final Comparator<RawDetection> detectionTYComparator;

    public CoralDetectorReal() {
        // TODO: confirm what min and max ty are and what the distance is for each
        distanceMap = new InterpolatingDoubleTreeMap();
        distanceMap.put(0.0, Inches.of(2).in(Meters));
        distanceMap.put(640.0, Inches.of(36).in(Meters));

        sortedDetections = new ArrayList<>();
        detectionTYComparator = (a, b) -> Double.compare(a.tync, b.tync);
    }

    @Override
    public Pose2d getCoralPose(Pose2d robotPose, RawDetection[] detections) {
        if (detections == null || detections.length == 0) {
            return null;
        }

        sortedDetections.clear();
        for (RawDetection detection : detections) {
            sortedDetections.add(detection);
        }
        sortedDetections.sort(detectionTYComparator);

        Pose2d basePose = robotPose;
        // Pose2d basePose = robotPose.transformBy(VisionSubsystem.ROBOT_TO_INTAKE_CAMERA);

        for (RawDetection detection : sortedDetections) {
            double distanceMeters = distanceMap.get(detection.tync);
            double degrees = detection.txnc;

            // TODO: double check if x or y is the correct value for distanceMeters
            Transform2d coralTransform = new Transform2d(0, distanceMeters, Rotation2d.fromDegrees(degrees));
            Pose2d coralPose = basePose.transformBy(coralTransform);

            // TODO: reject coralPoses that are outside the field or (perhaps) where the stacked coral are
            return coralPose;
        }
        return null;
    }
}
