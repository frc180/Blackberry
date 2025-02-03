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

public abstract class Auto {

    static final Transform2d hpStationTransform = new Transform2d(0, 0, Rotation2d.fromDegrees(150));

    public static boolean coralIntaking = false;
    
    public static void init() {
        coralIntaking = false;
    }

    public static boolean isCoralIntaking() {
        return coralIntaking;
    }

    public static Command coralIntake() {
        return Commands.runOnce(() -> coralIntaking = true);
    }

    public static Command stopCoralIntake() {
        return Commands.runOnce(() -> coralIntaking = false);
    }

    // TODO: add version of this that uses pathplanner to not hit reef when coming from far tags
    public static Command driveToHPStation() {
        return new DriveToPose(RobotContainer.instance.drivetrain, () -> {
                                    Pose2d hpStation = Robot.isBlue() ? SimLogic.blueHPCoralPose : SimLogic.redHPCoralPose;
                                    return hpStation.transformBy(hpStationTransform);
                                });
    }

    public static Command driveToHPStationFar() {
        DrivetrainSubsystem drivetrain = RobotContainer.instance.drivetrain;
        return Commands.defer(() -> {
            double sign = Robot.isBlue() ? 1 : -1;

            Pose2d hpStation = new Pose2d((Robot.isBlue() ? SimLogic.blueHPCoralPose : SimLogic.redHPCoralPose).getTranslation(), Rotation2d.kZero);
            Translation2d start = hpStation.getTranslation().plus(new Translation2d(2 * sign, -0.5 * sign));
            List<Pose2d> pathA = List.of(
                new Pose2d(drivetrain.getPose().getTranslation(), Rotation2d.fromDegrees(90 * sign)),
                new Pose2d(start, Rotation2d.fromDegrees(0)),
                new Pose2d(start, Rotation2d.fromDegrees(Robot.isBlue() ? (160 - 180) : 160))
            );
            return drivePath(pathA, 0, true);
        }, Set.of(drivetrain));
    }

    public static Command driveToCoral() {
        DrivetrainSubsystem drivetrain = RobotContainer.instance.drivetrain;
        VisionSubsystem vision = RobotContainer.instance.vision;

        return new DriveToPose(drivetrain, () -> {
            Pose2d robotPose = drivetrain.getPose();
            Pose2d coralPose = vision.getCoralPose();
            if (coralPose == null) return robotPose;

            Translation2d robotTranslation = robotPose.getTranslation();
            Translation2d coralTranslation = coralPose.getTranslation();

            // Calculate the angle to the coral
            double angleToCoral = Math.atan2(coralTranslation.getY() - robotTranslation.getY(), coralTranslation.getX() - robotTranslation.getX());

            return new Pose2d(coralPose.getTranslation(), new Rotation2d(angleToCoral + Math.PI));
        }).withDynamicTarget(true);
    }

    // TODO: This doesn't work great if the reef is not within a straight line from the robot position.
    // This method should be updated to use use Pathplanner paths (either all of the time, or when deemed necessary)
    // to move the robot most of the way there without collisions, then switch to the DriveToPose command to finish the job.
    public static Command driveToReefWithCoral() {
        return new DriveToPose(RobotContainer.instance.drivetrain, () -> Robot.nextAutoCoralScoringPosition().getPose())
                    .withPoseTargetType(PoseTarget.REEF)
                    .alongWith(Auto.stopCoralIntake());
    }

    public static Command drivePath(List<Pose2d> path, double endVel, boolean preventFlipping) {
        int lastIndex = path.size() - 1;
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(path.subList(0, lastIndex));
        return RobotContainer.instance.drivetrain.followPath(endVel, path.get(lastIndex), preventFlipping, waypoints);
    }
}
