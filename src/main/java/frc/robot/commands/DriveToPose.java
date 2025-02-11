package frc.robot.commands;

import java.util.function.Supplier;

import com.spamrobotics.util.Helpers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.HeadingTarget;
import frc.robot.subsystems.DrivetrainSubsystem.PoseTarget;

public class DriveToPose extends Command {
  
    private final DrivetrainSubsystem drivetrain;
    private final ChassisSpeeds noneSpeeds = new ChassisSpeeds(0, 0, 0);

    private Supplier<Pose2d> targetPoseSupplier = null;
    private Pose2d currentPose = null;
    private Pose2d targetPose = null;
    private Supplier<ChassisSpeeds> additionalSpeedsSupplier = null;
    private Double xEndTolerance = null;
    private Double yEndTolerance = null;
    private Double headingEndTolerance = null;
    private boolean dynamicTarget = false;
    private boolean holdWithinTolerance = false;
    private Supplier<Boolean> finishCriteria = null;
    private PoseTarget poseTargetType = PoseTarget.STANDARD;
    private double maxSpeed = 1.0;

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
 
    public DriveToPose withHoldWithinTolerance(boolean stopWithinTarget) {
        this.holdWithinTolerance = stopWithinTarget;
        return this;
    }

    public DriveToPose withPoseTargetType(PoseTarget target) {
        poseTargetType = target;
        return this;
    }

    public DriveToPose withMaxSpeed(double maxSpeedPercentage) {
        maxSpeed = maxSpeedPercentage;
        return this;
    }

    @Override
    public void initialize() {
        drivetrain.resetPIDs(HeadingTarget.POSE);
        if (!dynamicTarget && targetPoseSupplier != null) {
            targetPose = targetPoseSupplier.get();
        }
        drivetrain.setPoseTargetType(poseTargetType);
        drivetrain.setTargetPose(targetPose);
        if (poseTargetType == PoseTarget.REEF) {
            drivetrain.setTargetPoseTag(RobotContainer.instance.vision.getReefTagFromPose(targetPose));
        } else {
            drivetrain.setTargetPoseTag(-1);
        }
    }

    @Override
    public void execute() {
        currentPose = drivetrain.getPose();
        //this is using the current pose as the one from the drivetrain, not the one that is estimated by the limelight

        if (dynamicTarget && targetPoseSupplier != null) {
            targetPose = targetPoseSupplier.get();
        }
        drivetrain.setTargetPose(targetPose);

        if (holdWithinTolerance) {
            boolean inTolerances = Helpers.withinTolerance(currentPose, targetPose, xEndTolerance, yEndTolerance, headingEndTolerance);
            if (inTolerances) {
                drivetrain.drive(noneSpeeds);
                return;
            }
        }

        ChassisSpeeds speeds = drivetrain.calculateChassisSpeeds(currentPose, targetPose);
        if (additionalSpeedsSupplier != null) {
            Helpers.addChassisSpeedsOverwrite(speeds, additionalSpeedsSupplier.get());
        }
        double maxSpeedMeters = DrivetrainSubsystem.MAX_SPEED * maxSpeed;
        speeds.vxMetersPerSecond = MathUtil.clamp(speeds.vxMetersPerSecond, -maxSpeedMeters, maxSpeedMeters);
        speeds.vyMetersPerSecond = MathUtil.clamp(speeds.vyMetersPerSecond, -maxSpeedMeters, maxSpeedMeters);
        drivetrain.drive(speeds);
    }

    @Override
    public boolean isFinished() {
        // If the desired behavior is to hold position within the tolerance window, instead
        // of exiting the command, then we should not ever exist by ourselves.
        if (holdWithinTolerance) {
            return false;
        }
        // If no end tolerances are set, then the command will never finish on its own
        if (xEndTolerance == null && yEndTolerance == null && headingEndTolerance == null && finishCriteria == null) {
            return false;
        }

        boolean finishCriteriaSatisfied = finishCriteria == null || finishCriteria.get();
        boolean inTolerances = Helpers.withinTolerance(currentPose, targetPose, xEndTolerance, yEndTolerance, headingEndTolerance);

        return inTolerances && finishCriteriaSatisfied;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(noneSpeeds);
        drivetrain.setTargetPose(null);
        drivetrain.setPoseTargetType(PoseTarget.STANDARD);
    }
}
