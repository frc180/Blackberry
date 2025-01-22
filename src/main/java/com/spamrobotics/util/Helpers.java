package com.spamrobotics.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Helpers {
    
    public static ChassisSpeeds addChassisSpeeds(ChassisSpeeds a, ChassisSpeeds b) {
        return new ChassisSpeeds(
            capValue(a.vxMetersPerSecond + b.vxMetersPerSecond, DrivetrainSubsystem.MAX_SPEED),
            capValue(a.vyMetersPerSecond + b.vyMetersPerSecond, DrivetrainSubsystem.MAX_SPEED),
            capValue(a.omegaRadiansPerSecond + b.omegaRadiansPerSecond, DrivetrainSubsystem.MAX_ANGULAR_RATE)
        );
    }

    public static ChassisSpeeds addChassisSpeedsOverwrite(ChassisSpeeds a, ChassisSpeeds b) {
        a.vxMetersPerSecond = capValue(a.vxMetersPerSecond + b.vxMetersPerSecond, DrivetrainSubsystem.MAX_SPEED);
        a.vyMetersPerSecond = capValue(a.vyMetersPerSecond + b.vyMetersPerSecond, DrivetrainSubsystem.MAX_SPEED);
        a.omegaRadiansPerSecond = capValue(a.omegaRadiansPerSecond + b.omegaRadiansPerSecond, DrivetrainSubsystem.MAX_ANGULAR_RATE);
        return a;
    }

    public static double capValue(double value, double max) {
        if (Math.abs(value) > max) return max * (value / Math.abs(value));
        return value;
    }

    public static boolean withinTolerance(Pose2d pose, Pose2d targetPose, Double xMeters, Double yMeters, Double degrees) {
        // If one or more of the poses don't exist, we can't be within tolerance
        if (pose == null || targetPose == null) {
            return false;
        }
        
        Transform2d diff = null;
        if (xMeters != null || yMeters != null) {
            diff = pose.minus(targetPose);
        }

        boolean xSatisfied = xMeters == null || Math.abs(diff.getX()) <= xMeters;
        boolean ySatisfied = yMeters == null || Math.abs(diff.getY()) <= yMeters;

        // If we haven't met the x and y criteria, don't bother calculating any further
        if (!xSatisfied || !ySatisfied) {
            return false;
        }

        boolean headingSatisfied;
        if (degrees == null) {
            headingSatisfied = true;
        } else {
            double poseDegrees = MathUtil.inputModulus(pose.getRotation().getDegrees(), -180, 180);
            double targetPoseDegrees = MathUtil.inputModulus(targetPose.getRotation().getDegrees(), -180, 180);
            double degreesDiff = MathUtil.inputModulus(poseDegrees - targetPoseDegrees, -180, 180);
            headingSatisfied = Math.abs(degreesDiff) <= degrees;
        }
        return headingSatisfied;
    }
}
