package frc.robot.subsystems.vision;

import org.ironmaple.simulation.SimulatedArena;
import com.spamrobotics.util.Helpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.LimelightHelpers.RawDetection;
import frc.robot.util.simulation.SimLogic;

/**
 * A coral detector that returns poses based off simulation, ignoring any detections passed to it.
 */
public class CoralDetectorSim implements CoralDetector {

    private final double detectionDistance;
    private final boolean useFOV;

    private double fovDegrees = 82; // Limelight 4 FOV

    /**
     * Create a new CoralDetectorSim with the given detection distance.
     * @param detectionMeters The maximum distance in meters the robot can be from the coral and still detect it.
     * @param useFOV Whether to use the robot's field of view to determine if the coral is in sight.
     */
    public CoralDetectorSim(double detectionMeters, boolean useFOV) {
        detectionDistance = detectionMeters;
        this.useFOV = useFOV;
    }

    @Override
    public Pose2d getCoralPose(Pose2d robotPose, RawDetection[] _detections) {
        if (robotPose == null) return null;

        if (RobotContainer.MAPLESIM) {
            return getCoralPoseMapleSim(robotPose);
        } else {
            return getCoralPoseBasic(robotPose);
        }
    }

    private Pose2d getCoralPoseBasic(Pose2d robotPose) {
        Pose2d coralPose = Robot.isBlue() ? SimLogic.blueHPCoralPose : SimLogic.redHPCoralPose;

        if (useFOV && Helpers.poseWithinPOV(robotPose, coralPose, fovDegrees, detectionDistance)) {
            return coralPose;
        } else if (!useFOV && robotPose.getTranslation().getDistance(coralPose.getTranslation()) <= detectionDistance) {
            return coralPose;
        }

        return null;
    }

    /**
     * Selects the closest coral to the robot within its field of view.
     */
    private Pose2d getCoralPoseMapleSim(Pose2d robotPose) {
        Pose3d[] corals = SimulatedArena.getInstance().getGamePiecesArrayByType("Coral");
        Pose2d bestCoral = null;
        double bestDistance = Double.MAX_VALUE;
        for (int i = 0; i < corals.length; i++) {
            // Filter out upright coral
            if (corals[i].getRotation().getY() > 0.2) continue;

            Pose2d coral = corals[i].toPose2d();
            if (Helpers.poseWithinPOV(robotPose, coral, fovDegrees, detectionDistance)) {
                double distance = robotPose.getTranslation().getDistance(coral.getTranslation());
                if (distance < bestDistance) {
                    if (!CoralDetector.isValid(coral)) {
                        continue;
                    }

                    bestCoral = coral;
                    bestDistance = distance;
                }
            }
        }

        if (bestCoral == null) return null;
        double noise = 0.1;
        return bestCoral.transformBy(new Transform2d(randomRange(-noise, noise), randomRange(-noise, noise), Rotation2d.kZero));
    }

    private double randomRange(double min, double max) {
        return Math.random() * (max - min) + min;
    }
}
