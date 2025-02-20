package frc.robot.commands;

import java.util.function.Function;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem.PoseTarget;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

/**
 * Helper class to simplify generating DriveToPose commands for coral scoring positions.
 */
public class DriveToCoralPose extends DriveToPose {

    /**
     * Create a new DriveToCoralPose command.
     * @param tagSupplier Supplier for the ID of the AprilTag to target.
     * @param tagToPoseFunction Function to convert an AprilTag ID to a Pose2d.
     */
    public DriveToCoralPose(Supplier<Integer> tagSupplier, Function<Integer, Pose2d> tagToPoseFunction) {
        super(RobotContainer.instance.drivetrain, () -> tagToPoseFunction.apply(tagSupplier.get()));
        withPoseTargetType(PoseTarget.REEF);
        withTargetPoseTag(tagSupplier);
        // withIntermediatePoses(CORAL_INTERMEDIATE);
    }

    // WIP nicer pathing to prevent arm collisions with reef or algae
    private static Function<Pose2d, Pose2d> CORAL_INTERMEDIATE = (Pose2d target) -> {
        ElevatorSubsystem elevator = RobotContainer.instance.elevator;
        Pose2d robotPose = RobotContainer.instance.drivetrain.getPose();

        // double dist = Math.abs(robotPose.getTranslation().getDistance(target.getTranslation()));
        double offset = 0;

        // if (dist > 1.1) {
        //     offset = -0.7;
        // }

        if (!elevator.isElevatorInScoringPosition()) {
            offset = -0.1;
        }

        if (offset != 0) {
            target = target.transformBy(new Transform2d(offset, 0, Rotation2d.kZero));
        }

        return target;
    };
}
