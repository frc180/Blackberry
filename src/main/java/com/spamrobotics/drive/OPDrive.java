package com.spamrobotics.drive;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.spamrobotics.util.Helpers;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.HeadingTarget;

/**
 * Implements the driving strategy described in this post from 2056:
 * https://www.chiefdelphi.com/t/team-2056-op-robotics-2025-technical-binder-release/502550/36?u=ryan_s
 */
public class OPDrive implements DriveStrategy {

    final DrivetrainSubsystem drivetrain;
    final SlewRateLimiter rateLimiter;

    final PIDController translationPID = new PIDController(5, 0, 0);
    final double[] outputs = new double[] { 0, 0, 0 };

    boolean resetting = true;

    public OPDrive(DrivetrainSubsystem drivetrain, double rateLimitMeters) {
        this(drivetrain, rateLimitMeters, 99);
    }

    public OPDrive(DrivetrainSubsystem drivetrain, double accelRateMPS, double decelRateMPS) {
        this.drivetrain = drivetrain;
        rateLimiter = new SlewRateLimiter(decelRateMPS, -accelRateMPS, 0.0);
    }

    @Override
    public void reset() {
        resetting = true;
    }

    @Override
    public ChassisSpeeds drive(Pose2d currentPose, Pose2d endPose, TrapezoidProfile profile, Constraints constraints) {
        calculateOutputs(currentPose, endPose, constraints.maxVelocity);
        return ChassisSpeeds.fromFieldRelativeSpeeds(outputs[0], outputs[1], outputs[2], currentPose.getRotation());
    }

    @Override
    public SwerveRequest.RobotCentric apply(SwerveRequest.RobotCentric request, Pose2d currentPose, Pose2d endPose, double maxVelocity) {
        calculateOutputs(currentPose, endPose, maxVelocity);
        Helpers.fromFieldRelativeSpeeds(outputs, currentPose.getRotation());
        return request.withVelocityX(outputs[0])
                      .withVelocityY(outputs[1])
                      .withRotationalRate(outputs[2]);
    }

    private void calculateOutputs(Pose2d currentPose, Pose2d endPose, double maxVelocity) {
        if (resetting) {
            translationPID.reset();
            drivetrain.resetHeadingPID(HeadingTarget.POSE);
            rateLimiter.reset(MathUtil.clamp(
                Helpers.velocityTowards(currentPose, drivetrain.getFieldRelativeSpeeds(), endPose.getTranslation()),
                -maxVelocity, 
                0
            ));

            resetting = false;
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

        xOutput = MathUtil.clamp(xOutput, -maxVelocity, maxVelocity);
        yOutput = MathUtil.clamp(yOutput, -maxVelocity, maxVelocity);

        double thetaOutput = drivetrain.calculateHeadingPID(currentPose.getRotation(), endPose.getRotation().getDegrees());

        outputs[0] = xOutput;
        outputs[1] = yOutput;
        outputs[2] = thetaOutput;
    }
}
