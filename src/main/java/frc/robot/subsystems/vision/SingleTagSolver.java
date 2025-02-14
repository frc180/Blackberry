package frc.robot.subsystems.vision;

import java.util.Optional;

import com.spamrobotics.util.Helpers;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer;
import frc.robot.util.LimelightHelpers.RawFiducial;

/**
 * This file is heavily WIP and not being used for anything yet
 */

// Based on https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2025-build-thread/477314/85
public class SingleTagSolver {
    
    // https://github.com/Mechanical-Advantage/RobotCode2025Public/blob/114304798667f01c27888443c8a2be61d25f3706/src/main/java/org/littletonrobotics/frc2025/RobotState.java#L121
    public Pose2d getPose(double timestamp, RawFiducial tag, Transform3d robotToCamera) {
        AprilTagFieldLayout fieldLayout = RobotContainer.instance.vision.aprilTagFieldLayout;
        // TODO: Optimization - we should pre-cache all 2D tag poses
        Optional<Pose3d> optionalTagPose = fieldLayout.getTagPose(tag.id);
        if (optionalTagPose.isEmpty()) {
            return null;
        }

        Pose3d tagPose = optionalTagPose.get();
        Pose2d tagPose2d = tagPose.toPose2d();
        double tx = Units.degreesToRadians(tag.txnc); 
        double ty = Units.degreesToRadians(-tag.tync);
        

        // Pose3d cameraPose = Vision.cameraPoses[observation.camera()];
        Pose3d cameraPose = new Pose3d().transformBy(robotToCamera);

        // Get odometry based pose at timestamp
        Pose2d sample = RobotContainer.instance.drivetrain.getBufferPose(timestamp);
        if (sample == null) {
            return null;
        }

        Rotation2d robotRotation = sample.getRotation();
        Rotation2d gyroAngle = RobotContainer.instance.drivetrain.getGyroscopeRotation();

        // Use 3D distance and tag angles to find robot pose
        Translation2d camToTagTranslation =
            new Pose3d(Translation3d.kZero, new Rotation3d(0, ty, -tx))
                .transformBy(
                    new Transform3d(new Translation3d(tag.distToCamera, 0, 0), Rotation3d.kZero))
                .getTranslation()
                .rotateBy(new Rotation3d(0, cameraPose.getRotation().getY(), 0))
                .toTranslation2d();

        Rotation2d camToTagRotation =
            robotRotation.plus(
                cameraPose.toPose2d().getRotation().plus(camToTagTranslation.getAngle()));

        Translation2d fieldToCameraTranslation =
            new Pose2d(tagPose2d.getTranslation(), camToTagRotation.plus(Rotation2d.kPi))
                .transformBy(Helpers.toTransform2d(camToTagTranslation.getNorm(), 0.0))
                .getTranslation();

        Pose2d robotPose =
            new Pose2d(
                    fieldToCameraTranslation, robotRotation.plus(cameraPose.toPose2d().getRotation()))
                .transformBy(new Transform2d(cameraPose.toPose2d(), Pose2d.kZero));
        // Use gyro angle at time for robot rotation
        // robotPose = new Pose2d(robotPose.getTranslation(), robotRotation);
        
        return robotPose;
    }
}
