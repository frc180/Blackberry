package com.spamrobotics.drive;

import com.spamrobotics.util.Helpers;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

@Logged
public class ProfiledLookaheadDrive extends DriveStrategy {

    final DrivetrainSubsystem drivetrain;
    final TrapezoidProfile profile;
    final PIDController xPid, yPid;
    final SimpleMotorFeedforward xyFeedforward;

    PIDController translationPid = null;
    SlewRateLimiter rateLimiter = null;

    final State startState = new State(0, 0);
    final State goalState = new State(0, 0);
    Pose2d poseStart = null;
    State previousSetpoint = null;
    private Pose2d profiledIntermediatePose = Pose2d.kZero;

    public ProfiledLookaheadDrive(DrivetrainSubsystem drivetrain, TrapezoidProfile profile, PIDController xPid, PIDController yPid, 
                                  SimpleMotorFeedforward xyFeedforward) {
        this.drivetrain = drivetrain;
        this.profile = profile;
        this.xPid = xPid;
        this.yPid = yPid;
        this.xyFeedforward = xyFeedforward;
    }

    public ProfiledLookaheadDrive withTranslationPid(double rateLimit, double p, double i, double d) {
        translationPid = new PIDController(p, i, d);
        rateLimiter = new SlewRateLimiter(999999, -rateLimit, 0.0);
        return this;
    }

    @Override
    public void reset() {
        xPid.reset();
        yPid.reset();
        poseStart = null;
    }

    @Override
    protected void calculateOutputs(Pose2d currentPose, Pose2d endPose, double maxVelocity) {
        boolean replanning = poseStart != null;
        
        if (!replanning) {
            xPid.reset();
            yPid.reset();
            if (translationPid != null) {
                translationPid.reset();
                rateLimiter.reset(MathUtil.clamp(
                    Helpers.velocityTowards(currentPose, drivetrain.getFieldRelativeSpeeds(), endPose.getTranslation()),
                    -maxVelocity, 
                    0
                ));
            }
        }
        poseStart = currentPose;

        double normDirX = endPose.getX() - poseStart.getX();
        double normDirY = endPose.getY() - poseStart.getY();
        double distance = Math.hypot(normDirX, normDirY);

        normDirX /= (distance + 0.001);
        normDirY /= (distance + 0.001);

        
        startState.position = distance;
        if (replanning) {
            startState.velocity = -drivetrain.getVelocity();
        } else {
            startState.velocity = MathUtil.clamp(
                Helpers.velocityTowards(poseStart, drivetrain.getFieldRelativeSpeeds(), endPose.getTranslation()),
                -maxVelocity, 
                0
            );
        }

        // Calculate the setpoint (i.e. current distance along the path) we should be targeting
        State setpoint = profile.calculate(Constants.LOOP_TIME * 6, startState, goalState); // experiment: try * 5
        Translation2d setpointTarget = endPose.getTranslation().interpolate(poseStart.getTranslation(), setpoint.position / distance);
        
        // For logging only
        profiledIntermediatePose = new Pose2d(setpointTarget, endPose.getRotation());

        // Use normal PIDs to calculate the feedback for the X and Y axes to reach the setpoint position
        double xOutput = xPid.calculate(currentPose.getX(), setpointTarget.getX());
        double yOutput = yPid.calculate(currentPose.getY(), setpointTarget.getY());

        // Use feedforward for the X and Y axes to better reach the setpoint speed
        double velocity = setpoint.velocity;
        if (translationPid != null) {
            velocity += translationPid.calculate(distance, 0);
            velocity = rateLimiter.calculate(velocity);
        }
        double xTargetVelocity = normDirX * -velocity;
        double yTargetVelocity = normDirY * -velocity;
        xOutput += xyFeedforward.calculate(xTargetVelocity);
        yOutput += xyFeedforward.calculate(yTargetVelocity);

        // Ensure X and Y outputs are within the max velocity constraints (note: this stops the PID from helping catch up if we're behind)
        xOutput = MathUtil.clamp(xOutput, -maxVelocity, maxVelocity);
        yOutput = MathUtil.clamp(yOutput, -maxVelocity, maxVelocity);

        double thetaOutput = drivetrain.calculateHeadingPID(currentPose.getRotation(), endPose.getRotation().getDegrees());

        outputs[0] = xOutput;
        outputs[1] = yOutput;
        outputs[2] = thetaOutput;
    }
}
