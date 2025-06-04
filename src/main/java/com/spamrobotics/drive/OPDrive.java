package com.spamrobotics.drive;

import com.spamrobotics.util.Helpers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.subsystems.DrivetrainSubsystem;

public class OPDrive implements AutoDriveStrategy {

    final DrivetrainSubsystem drivetrain;
    final SlewRateLimiter rateLimiter;

    final PIDController translationPID = new PIDController(5, 0, 0);

    boolean shouldReset = true;

    public OPDrive(DrivetrainSubsystem drivetrain, double rateLimitMeters) {
        this.drivetrain = drivetrain;

        rateLimiter = new SlewRateLimiter(99, -rateLimitMeters, 0.0);
    }

    @Override
    public void reset() {
        shouldReset = true;
    }

    @Override
    public ChassisSpeeds drive(Pose2d currentPose, Pose2d endPose, TrapezoidProfile profile, Constraints constraints) {

        if (shouldReset) {
            translationPID.reset();
            rateLimiter.reset(MathUtil.clamp(
                Helpers.velocityTowards(currentPose, drivetrain.getFieldRelativeSpeeds(), endPose.getTranslation()),
                -constraints.maxVelocity, 
                0
            ));

            shouldReset = false;
        }

        double normDirX = endPose.getX() - currentPose.getX();
        double normDirY = endPose.getY() - currentPose.getY();
        double distance = Math.hypot(normDirX, normDirY);

        normDirX /= (distance + 0.001);
        normDirY /= (distance + 0.001);


        double translationOutput = translationPID.calculate(distance, 0);
        translationOutput = rateLimiter.calculate(translationOutput);

        double xOutput = normDirX * -translationOutput;
        double yOutput = normDirY * -translationOutput;

        xOutput = MathUtil.clamp(xOutput, -constraints.maxVelocity, constraints.maxVelocity);
        yOutput = MathUtil.clamp(yOutput, -constraints.maxVelocity, constraints.maxVelocity);

        double thetaOutput = drivetrain.calculateHeadingPID(currentPose.getRotation(), endPose.getRotation().getDegrees());

        return ChassisSpeeds.fromFieldRelativeSpeeds(xOutput, yOutput, thetaOutput, currentPose.getRotation());

    }
}
