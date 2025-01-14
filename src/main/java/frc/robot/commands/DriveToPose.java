package frc.robot.commands;

import java.util.function.Supplier;
import com.spamrobotics.util.Helpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.HeadingTarget;

public class DriveToPose extends Command {
  
    private final DrivetrainSubsystem drivetrain;

    private Supplier<Pose2d> targetPoseSupplier = null;
    private Pose2d currentPose = null;
    private Pose2d targetPose = null;
    private Supplier<ChassisSpeeds> additionalSpeedsSupplier = null;
    private Double xEndTolerance = null;
    private Double yEndTolerance = null;
    private Double headingEndTolerance = null;
    private boolean dynamicTarget = false;
    private Supplier<Boolean> finishCriteria = null;

    public DriveToPose(DrivetrainSubsystem drivetrainSubsystem, Supplier<Pose2d> targetPoseSupplier) {
        drivetrain = drivetrainSubsystem;
        this.targetPoseSupplier = targetPoseSupplier;
        addRequirements(drivetrainSubsystem);
    }

    public DriveToPose(DrivetrainSubsystem drivetrainSubsystem, Pose2d targetPose) {
        drivetrain = drivetrainSubsystem;
        this.targetPose = targetPose;
        addRequirements(drivetrainSubsystem);
    }

    public DriveToPose withXEndTolerance(double toleranceMeters) {
        this.xEndTolerance = toleranceMeters;
        return this;
    }

    public DriveToPose withYEndTolerance(double toleranceMeters) {
        this.yEndTolerance = toleranceMeters;
        return this;
    }

    public DriveToPose withXYEndTolerance(double toleranceMeters) {
        return withXEndTolerance(toleranceMeters).withYEndTolerance(toleranceMeters);
    }

    public DriveToPose withHeadingEndTolerance(double toleranceDegrees) {
        this.headingEndTolerance = toleranceDegrees;
        return this;
    }

    public DriveToPose withAdditionalSpeeds(Supplier<ChassisSpeeds> additionalSpeedsSupplier) {
        this.additionalSpeedsSupplier = additionalSpeedsSupplier;
        return this;
    }

    public DriveToPose withDynamicTarget(boolean dynamicTarget) {
        this.dynamicTarget = dynamicTarget;
        return this;
    }

    public DriveToPose withFinishCriteria(Supplier<Boolean> finishCriteria) {
        this.finishCriteria = finishCriteria;
        return this;
    }

    @Override
    public void initialize() {
        drivetrain.resetPIDs(HeadingTarget.POSE);
        if (!dynamicTarget && targetPoseSupplier != null) {
            targetPose = targetPoseSupplier.get();
        }
    }

    @Override
    public void execute() {
        currentPose = drivetrain.getPose();
        if (dynamicTarget && targetPoseSupplier != null) {
            targetPose = targetPoseSupplier.get();
        }

        ChassisSpeeds speeds = drivetrain.calculateChassisSpeeds(currentPose, targetPose);
        if (additionalSpeedsSupplier != null) {
            Helpers.addChassisSpeedsOverwrite(speeds, additionalSpeedsSupplier.get());
        }
        drivetrain.drive(speeds);
    }

    @Override
    public boolean isFinished() {
        // If no end tolerances are set, then the command will never finish on its own
        if (xEndTolerance == null && yEndTolerance == null && headingEndTolerance == null && finishCriteria == null) {
            return false;
        }
        
        Transform2d diff = null;
        if (xEndTolerance != null || yEndTolerance != null) {
            diff = currentPose.minus(targetPose);
        }

        boolean xSatisfied = xEndTolerance == null || Math.abs(diff.getX()) <= xEndTolerance;
        boolean ySatisfied = yEndTolerance == null || Math.abs(diff.getY()) <= yEndTolerance;

        boolean headingSatisfied;
        if (headingEndTolerance == null) {
            headingSatisfied = true;
        } else {
            headingSatisfied = Math.abs(currentPose.getRotation().getDegrees() - targetPose.getRotation().getDegrees()) <= headingEndTolerance;
        }

        boolean finishCriteriaSatisfied = finishCriteria == null || finishCriteria.get();

        return xSatisfied && ySatisfied && headingSatisfied & finishCriteriaSatisfied;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0, 0, 0));
    }
}
