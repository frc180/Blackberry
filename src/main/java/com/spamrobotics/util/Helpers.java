package com.spamrobotics.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
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

    public static ChassisSpeeds rotateChassisSpeeds(ChassisSpeeds speeds, Rotation2d rotation) {
        Translation2d rotated = new Translation2d(speeds.vxMetersPerSecond, speeds.vxMetersPerSecond).rotateBy(rotation);
        return new ChassisSpeeds(rotated.getX(), rotated.getY(), speeds.omegaRadiansPerSecond);
    }

    // Untested, should help with limiting acceleration dynamically
    public static ChassisSpeeds rateLimit(ChassisSpeeds prev, ChassisSpeeds next, double maxDeltaV) {
        ChassisSpeeds diff = next.minus(prev);
        double diffSqMag = diff.vxMetersPerSecond * diff.vxMetersPerSecond + diff.vyMetersPerSecond * diff.vyMetersPerSecond;
        if (diffSqMag <= maxDeltaV * maxDeltaV) {
          return next;
        }
        return prev.plus(diff.times(maxDeltaV / Math.sqrt(diffSqMag)));
      }

    public static double perpendicularLineLength(Translation2d point, Translation2d vectorStart, Translation2d vectorEnd) {
        double numerator = Math.abs((vectorEnd.getY() - vectorStart.getY()) * point.getX() - 
                                    (vectorEnd.getX() - vectorStart.getX()) * point.getY() + 
                                    vectorEnd.getX() * vectorStart.getY() - 
                                    vectorEnd.getY() * vectorStart.getX());
        double denominator = vectorStart.getDistance(vectorEnd);
        return numerator / denominator;
    }

    public static double capValue(double value, double max) {
        if (Math.abs(value) > max) return max * (value / Math.abs(value));
        return value;
    }

    public static boolean withinTolerance(Pose2d pose, Pose2d targetPose, Distance xDistance, Distance yDistance, Angle angle) {
        Double xMeters = xDistance == null ? null : xDistance.in(Meters);
        Double yMeters = yDistance == null ? null : yDistance.in(Meters);
        Double degrees = angle == null ? null : angle.in(Degrees);
        return withinTolerance(pose, targetPose, xMeters, yMeters, degrees);
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

    public static Transform3d toTransform3d(Distance x, Distance y, Distance z, Rotation3d rotation) {
        return new Transform3d(x.in(Meters), y.in(Meters), z.in(Meters), rotation);
    }

    public static Transform2d toTransform2d(Distance x, Distance y, Angle angle) {
        return new Transform2d(x.in(Meters), y.in(Meters), Rotation2d.fromDegrees(angle.in(Degrees)));
    }

    public static Transform2d toTransform2d(Translation2d translation) {
        // return new Transform2d(translation, new Rotation2d());
        return new Transform2d(translation, Rotation2d.kZero);
    }

    public static Transform2d toTransform2d(double x, double y) {
        // return new Transform2d(x, y, new Rotation2d());
        return new Transform2d(x, y, Rotation2d.kZero);
    }

    // Helper methods from https://github.com/frc6995/Robot-2025/blob/main/src/main/java/frc/robot/subsystems/drive/Pathing.java#L18

    public static double velocityTowards(Pose2d currentPose, ChassisSpeeds fieldRelativeSpeeds, Translation2d targetTranslation) {
      return pointRelativeSpeeds(currentPose, fieldRelativeSpeeds, targetTranslation)
                .vxMetersPerSecond;
    }

    public static ChassisSpeeds pointRelativeSpeeds(Pose2d currentPose, ChassisSpeeds fieldRelativeSpeeds, Translation2d targetTranslation) {
      return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, headingTo(currentPose, targetTranslation));
    }

    public static Rotation2d headingTo(Pose2d currentPose, Translation2d target) {
        return target.minus(currentPose.getTranslation()).getAngle().minus(Rotation2d.kPi);
      }
}
