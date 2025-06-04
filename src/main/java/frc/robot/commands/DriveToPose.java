package frc.robot.commands;

import java.util.function.Function;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.HeadingTarget;
import frc.robot.subsystems.DrivetrainSubsystem.PoseTarget;

public class DriveToPose extends Command {
  
    private final DrivetrainSubsystem drivetrain;
    private final ChassisSpeeds noneSpeeds = new ChassisSpeeds(0, 0, 0);

    private Supplier<Pose2d> targetPoseSupplier = null;
    private Function<Integer, Pose2d> tagToPoseFunction = null;
    private Pose2d currentPose = null;
    private Pose2d targetPose = null;
    private boolean dynamicTarget = false;
    private PoseTarget poseTargetType = PoseTarget.STANDARD;
    private double maxSpeed = 1.0;
    private Supplier<Integer> targetPoseTagSupplier = null;
    private Function<Pose2d, Pose2d> intermediatePoses = null;

    private int targetPoseTag = -1;

    public DriveToPose(DrivetrainSubsystem drivetrainSubsystem, Function<Integer, Pose2d> tagToPoseFunction) {
        drivetrain = drivetrainSubsystem;
        this.tagToPoseFunction = tagToPoseFunction;
        addRequirements(drivetrainSubsystem);
    }

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

    public DriveToPose withDynamicTarget(boolean dynamicTarget) {
        this.dynamicTarget = dynamicTarget;
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

    public DriveToPose withTargetPoseTag(Supplier<Integer> targetPoseTagSupplier) {
        this.targetPoseTagSupplier = targetPoseTagSupplier;
        return this;
    }

    public DriveToPose withIntermediatePoses(Function<Pose2d, Pose2d> intermediatePoses) {
        this.intermediatePoses = intermediatePoses;
        return this;
    }

    @Override
    public void initialize() {
        targetPoseTag = targetPoseTagSupplier != null ? targetPoseTagSupplier.get() : -1;

        drivetrain.setTargetPoseTag(targetPoseTag);
        drivetrain.setPoseTargetType(poseTargetType);
        drivetrain.resetPIDs(HeadingTarget.POSE);

        if (targetPoseSupplier != null) {
            targetPose = targetPoseSupplier.get();
        }
        if (tagToPoseFunction != null) {
            targetPose = tagToPoseFunction.apply(targetPoseTag);
        }
        drivetrain.setTargetPose(targetPose);
    }

    @Override
    public void execute() {
        currentPose = drivetrain.getPose();

        if (dynamicTarget) {
            if (targetPoseSupplier != null) targetPose = targetPoseSupplier.get();
            if (tagToPoseFunction != null) targetPose = tagToPoseFunction.apply(targetPoseTag);  
        }
        drivetrain.setTargetPose(targetPose);
        Pose2d iterationTarget = targetPose;
        
        if (intermediatePoses != null) {
            iterationTarget = intermediatePoses.apply(targetPose);
        }
        drivetrain.setIntermediatePose(iterationTarget);

        final double maxSpeedMeters = DrivetrainSubsystem.MAX_SPEED * maxSpeed;
  
        // ChassisSpeeds speeds = drivetrain.driveToStrategy.drive(
        //     currentPose,
        //     iterationTarget,
        //     Math.min(drivetrain.driveToPoseConstraints.maxVelocity, maxSpeedMeters)
        // );
        // drivetrain.driveClosedLoop(speeds);

        // EXPERIMENTAL
        drivetrain.setControl(drivetrain.driveToStrategy.apply(
            drivetrain.closedLoopRobotCentric,
            currentPose,
            iterationTarget,
            Math.min(drivetrain.driveToPoseConstraints.maxVelocity, maxSpeedMeters)
        ));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(noneSpeeds);
        drivetrain.setTargetPose(null);
        drivetrain.setPoseTargetType(PoseTarget.STANDARD);
        drivetrain.setTargetHeading(null);
    }
}
