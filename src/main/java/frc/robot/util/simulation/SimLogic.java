package frc.robot.util.simulation;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public abstract class SimLogic {

    public static final Pose2d blueHPCoralPose = new Pose2d(1.56, 1.33, new Rotation2d());
    public static final Pose2d redHPCoralPose = new Pose2d(16.17, 1.33, new Rotation2d());

    public static boolean hasCoral = false;

    public static void spawnHumanPlayerCoral() {
        if (!RobotContainer.MAPLESIM) {
            return;
        }

        Pose2d coralPose = Robot.isBlue() ? blueHPCoralPose : redHPCoralPose;

        // Apply a random x, y, and rotation offset to the coral

        // generate a random x offset between -0.3 and 0.3 meters
        double xOffset = randomNumberPlusMinus(0.6);
        double yOffset = randomNumberPlusMinus(0.6);
        double rotationOffset = Math.random() * 360;

        Transform2d randomTransform = new Transform2d(xOffset, yOffset, Rotation2d.fromDegrees(rotationOffset));

        coralPose = coralPose.transformBy(randomTransform);

        SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(coralPose));
    }

    private static double randomNumberPlusMinus(double range) {
        return Math.random() * (range * 2) - range;
    }
}
