package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.VisionSubsystem;

public class CoralScoringPosition {

    public final int tag;
    public final int level;
    public final boolean isLeft;
    public final boolean blueAlliance;

    private Pose2d pose = null;

    public CoralScoringPosition(int tag, int level, boolean isLeft) {
        this.tag = tag;
        this.level = level;
        this.isLeft = isLeft;
        blueAlliance = VisionSubsystem.blueReefTags.contains(tag);
    }

    public Pose2d getPose() {
        if (pose == null) {
            VisionSubsystem vision = RobotContainer.instance.vision;
            pose = (isLeft ? vision.leftReefHashMap : vision.rightReefHashMap).get(tag);
        }

        return pose;
    }

    public CoralScoringPosition getFlippedIfNeeded() {
        if (Robot.isBlue() == blueAlliance) {
            return this;
        } else {
            return getFlipped();
        }
    }

    /**
     * Get the equivalent reef scoring position on the opposite alliance
     */
    public CoralScoringPosition getFlipped() {
        VisionSubsystem vision = RobotContainer.instance.vision;
        int flippedTag = blueAlliance ? vision.blueReefTagToRed(tag) : vision.redReefTagToBlue(tag);
        if (flippedTag == -1) {
            throw new RuntimeException("Tag " + tag + " (blue=" + blueAlliance + ") not found in opposite alliance");
        }
        return new CoralScoringPosition(flippedTag, level, isLeft);
    }
}