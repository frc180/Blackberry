package frc.robot.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This class is used to determine the closest reef scoring position to the robot's current position.
 */
public class ReefProximity {

    final List<Entry<Integer, Pose2d>> reefList;

    public ReefProximity(HashMap<Integer, Pose2d> leftReefHashMap, HashMap<Integer, Pose2d> rightReefHashMap) {
        reefList = new ArrayList<Entry<Integer, Pose2d>>();
        leftReefHashMap.entrySet().forEach(reefList::add);
        rightReefHashMap.entrySet().forEach(reefList::add);
    }

    public Entry<Integer, Pose2d> closestReefPose(Pose2d position, boolean blue) {
        double currentDistance = Double.MAX_VALUE;
        Entry<Integer, Pose2d> closest = null;

        for (var entry : reefList) {
            // Ensure the reef tag belongs to the same alliance as the robot, and skip if it doesn't
            boolean sameAlliance = blue ? VisionSubsystem.blueReefTags.contains(entry.getKey()) : VisionSubsystem.redReefTags.contains(entry.getKey());
            if (!sameAlliance) {
                continue;
            }

            double distance = position.getTranslation().getDistance(entry.getValue().getTranslation());
            if (distance < currentDistance) {
                currentDistance = distance;
                closest = entry;
            }
        }

        return closest;
    }
}
