package com.spamrobotics.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public interface DriveStrategy {

    public void reset();

    public ChassisSpeeds drive(Pose2d currentPose, Pose2d endPose, TrapezoidProfile profile, TrapezoidProfile.Constraints constraints);
}