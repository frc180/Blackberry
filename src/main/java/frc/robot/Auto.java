package frc.robot;

import java.util.List;
import java.util.Set;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.PoseTarget;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.simulation.SimLogic;

public final class Auto {

    private static final Transform2d HP_STATION_TRANSFORM = new Transform2d(0, 0, Rotation2d.fromDegrees(150));

    public static boolean coralIntaking = false;

    private Auto() {}
    
    public static void init() {
        coralIntaking = false;
    }

    public static boolean isCoralIntaking() {
        return coralIntaking;
    }

    public static Command startCoralIntake() {
        return Commands.runOnce(() -> coralIntaking = true);
    }

    public static Command stopCoralIntake() {
        return Commands.runOnce(() -> coralIntaking = false);
    }

    public static DriveToPose driveToHPStation() {
        return new DriveToPose(RobotContainer.instance.drivetrain, () -> {
                                    Pose2d hpStation = Robot.isBlue() ? SimLogic.blueHPCoralPose : SimLogic.redHPCoralPose;
                                    return hpStation.transformBy(HP_STATION_TRANSFORM);
                                });
    }

    public static Command driveToHPStationFar() {
        DrivetrainSubsystem drivetrain = RobotContainer.instance.drivetrain;
        return Commands.defer(() -> {
            double sign = Robot.isBlue() ? 1 : -1;

            Pose2d hpStation = new Pose2d((Robot.isBlue() ? SimLogic.blueHPCoralPose : SimLogic.redHPCoralPose).getTranslation(), Rotation2d.kZero);
            Translation2d end = hpStation.getTranslation().plus(new Translation2d(2 * sign, -0.5 * sign));
            List<Pose2d> pathA = List.of(
                new Pose2d(drivetrain.getPose().getTranslation(), Rotation2d.fromDegrees(90 * sign)),
                new Pose2d(end, Rotation2d.fromDegrees(0)),
                new Pose2d(end, Rotation2d.fromDegrees(160 - (Robot.isBlue() ? 180 : 0)))
            );
            return drivetrain.followPath(pathA, 0, true);
        }, Set.of(drivetrain));
    }

    public static DriveToPose driveToCoral() {
        DrivetrainSubsystem drivetrain = RobotContainer.instance.drivetrain;
        VisionSubsystem vision = RobotContainer.instance.vision;

        return new DriveToPose(drivetrain, () -> {
            Pose2d robotPose = drivetrain.getPose();
            Pose2d coralPose = vision.getCoralPose();
            if (coralPose == null) return robotPose;

            Translation2d robotTranslation = robotPose.getTranslation();
            Translation2d coralTranslation = coralPose.getTranslation();

            // Calculate the angle in radians to the coral
            double angleToCoral = Math.atan2(coralTranslation.getY() - robotTranslation.getY(), coralTranslation.getX() - robotTranslation.getX());

            return new Pose2d(coralPose.getTranslation(), new Rotation2d(angleToCoral + Math.PI));
        }).withDynamicTarget(true);
    }

    public static Command driveToReefWithCoral() {
        return new DriveToPose(RobotContainer.instance.drivetrain, () -> Robot.nextAutoCoralScoringPosition().getPose())
                    .withPoseTargetType(PoseTarget.REEF)
                    .alongWith(Auto.stopCoralIntake());
    }
}
