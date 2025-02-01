package frc.robot;

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

    public static Command driveToHPStation() {
        return new DriveToPose(RobotContainer.instance.drivetrain, () -> {
                                    Pose2d hpStation = Robot.isBlue() ? SimLogic.blueHPCoralPose : SimLogic.redHPCoralPose;
                                    return hpStation.transformBy(new Transform2d(0, 0, Rotation2d.k180deg));
                                });
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
}
