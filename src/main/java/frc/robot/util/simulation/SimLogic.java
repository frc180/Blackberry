package frc.robot.util.simulation;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public abstract class SimLogic {

    public static final Pose2d redHPCoralPose = new Pose2d(16.17, 1.33, new Rotation2d());
    public static final Pose2d blueHPCoralPose = FlippingUtil.flipFieldPose(redHPCoralPose);

    public static boolean intakeHasCoral = false;
    public static boolean armHasCoral = false;

    public static boolean robotHasCoral() {
        return intakeHasCoral || armHasCoral;
    }

    public static void spawnHumanPlayerCoral() {
        spawnHumanPlayerCoral(Robot.isBlue());
    }

    public static void spawnHumanPlayerCoral(boolean blue) {
        if (!RobotContainer.MAPLESIM) {
            return;
        }

        Pose2d coralPose = blue ? blueHPCoralPose : redHPCoralPose;

        // generate a random physical offset between -0.6 and 0.6 meters and a random rotation
        double xOffset = randomNumberPlusMinus(0.6);
        double yOffset = randomNumberPlusMinus(0.6);
        double rotationOffset = Math.random() * 360;
        Transform2d randomTransform = new Transform2d(xOffset, yOffset, Rotation2d.fromDegrees(rotationOffset));

        SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(coralPose.transformBy(randomTransform)));
    }

    private static double randomNumberPlusMinus(double range) {
        return Math.random() * (range * 2) - range;
    }
}
