package frc.robot;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.CoralPlacementCycle;
import frc.robot.subsystems.CoralPlacerSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public final class Auto {
    private static final double DISTANCE_PHASE0_METERS = 5.2;
    private static final double DISTANCE_PHASE1_METERS = 3;
    private static final double DRIVE_SPEED = 4.0;
    private static final double ROTATION_DEGREES = 162.0;
    private static final double ADDITIONAL_ROTATION_DEGREES = 30.0;
    private static final double TOLERANCE_DEGREES = 0.5;
    private static final double RAMP_DISTANCE = 1.5;
    private static final double MIN_SPEED = 0.2;
    private static final double DT = 0.02;

    private Auto() {}

    public static void init() {
    }

    public static Command getBlueLeftAuto(DrivetrainSubsystem drivetrain, CoralPlacerSubsystem placerSubsystem, Pose2d startPose) {
        Command drivingCommand = new Command() {
            private int phase = 0;
            private Pose2d phaseStartPose;
            private double initialHeadingDegrees;
            private double pauseStartTime;
            private double targetHeading;

            @Override
            public void initialize() {
                drivetrain.resetPose(startPose);
                phaseStartPose = drivetrain.getPose();
                Rotation2d initialHeading = phaseStartPose.getRotation();
                initialHeadingDegrees = initialHeading.getDegrees();
                drivetrain.resetHeadingPID(initialHeadingDegrees);
            }

            @Override
            public void execute() {
                Pose2d currentPose = drivetrain.getPose();
                double linearSpeed = DRIVE_SPEED;
                double vx = 0.0;
                double vy = 0.0;
                double omega = 0.0;

                if (phase == 0) {
                    double traveled = phaseStartPose.getX() - currentPose.getX();
                    double remaining = DISTANCE_PHASE0_METERS - traveled;
                    if (remaining < DRIVE_SPEED * DT) {
                        linearSpeed = remaining / DT;
                    } else if (remaining < RAMP_DISTANCE) {
                        linearSpeed = Math.max(MIN_SPEED, DRIVE_SPEED * (remaining / RAMP_DISTANCE));
                    }
                    double currentHeadingDegrees = currentPose.getRotation().getDegrees();
                    omega = drivetrain.calculateHeadingPID(currentHeadingDegrees, initialHeadingDegrees);
                    vx = -linearSpeed;
                } else if (phase == 2) {
                    double dx = currentPose.getX() - phaseStartPose.getX();
                    double dy = currentPose.getY() - phaseStartPose.getY();
                    double traveled = Math.sqrt(dx * dx + dy * dy);
                    double remaining = DISTANCE_PHASE1_METERS - traveled;
                    if (remaining < DRIVE_SPEED * DT) {
                        linearSpeed = remaining / DT;
                    } else if (remaining < RAMP_DISTANCE) {
                        linearSpeed = Math.max(MIN_SPEED, DRIVE_SPEED * (remaining / RAMP_DISTANCE));
                    }
                    double angularRate = Math.toRadians(ROTATION_DEGREES) / DISTANCE_PHASE1_METERS;
                    omega = angularRate * linearSpeed;
                    double angleRad = Math.toRadians(305);
                    vx = linearSpeed * Math.cos(angleRad);
                    vy = linearSpeed * Math.sin(angleRad);
                } else if (phase == 3) {
                    double currentHeadingDegrees = currentPose.getRotation().getDegrees();
                    omega = drivetrain.calculateHeadingPID(currentHeadingDegrees, targetHeading);
                }

                ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(vx, vy, omega);

                Rotation2d currentHeading = currentPose.getRotation();
                ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    fieldRelativeSpeeds.vxMetersPerSecond,
                    fieldRelativeSpeeds.vyMetersPerSecond,
                    fieldRelativeSpeeds.omegaRadiansPerSecond,
                    currentHeading
                );

                drivetrain.drive(robotRelativeSpeeds);
            }

            @Override
            public boolean isFinished() {
                Pose2d currentPose = drivetrain.getPose();

                if (phase == 0) {
                    if (phaseStartPose.getX() - currentPose.getX() >= DISTANCE_PHASE0_METERS) {
                        phase = 1;
                        pauseStartTime = Timer.getFPGATimestamp();
                        return false;
                    }
                } else if (phase == 1) {
                    double currentTime = Timer.getFPGATimestamp();
                    if (currentTime - pauseStartTime >= 2.0) {
                        phase = 2;
                        phaseStartPose = drivetrain.getPose();
                        return false;
                    }
                } else if (phase == 2) {
                    double dx = currentPose.getX() - phaseStartPose.getX();
                    double dy = currentPose.getY() - phaseStartPose.getY();
                    double distTraveled = Math.sqrt(dx * dx + dy * dy);
                    if (distTraveled >= DISTANCE_PHASE1_METERS) {
                        phase = 3;
                        targetHeading = currentPose.getRotation().getDegrees() + ADDITIONAL_ROTATION_DEGREES;
                        drivetrain.resetHeadingPID(targetHeading);
                        return false;
                    }
                } else if (phase == 3) {
                    double currentHeadingDegrees = currentPose.getRotation().getDegrees();
                    return Math.abs(currentHeadingDegrees - targetHeading) < TOLERANCE_DEGREES;
                }

                return false;
            }

            @Override
            public void end(boolean interrupted) {
                drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
            }

            @Override
            public Set<Subsystem> getRequirements() {
                return Set.of(drivetrain);
            }
        };

        return drivingCommand.andThen(new CoralPlacementCycle(placerSubsystem, Constants.Commands.CORAL_OUTTAKE_SPEED)).withTimeout(30.0);
    }
}