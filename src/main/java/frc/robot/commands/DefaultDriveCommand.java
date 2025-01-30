package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.HeadingTarget;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import com.spamrobotics.util.Helpers;
import com.spamrobotics.util.JoystickInputs;

public class DefaultDriveCommand extends Command {
    private final double coralAssistKp = 0.9;
    private final double coralAssistExponent = -3/2;
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final Supplier<JoystickInputs> m_joystickInputsSupplier;
    private final DoubleSupplier m_rotationSupplier;

    private boolean manuallyRotating;
    private Rotation2d gyroRotation;
    private HeadingTarget previousHeadingType = null;
    private double rotationSpeed = 0;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               Supplier<JoystickInputs> joystickInputsSupplier,
                               DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_joystickInputsSupplier = joystickInputsSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        manuallyRotating = false;
        m_drivetrainSubsystem.setTargetHeading(null);
        // m_drivetrainSubsystem.setTargetHeading(m_drivetrainSubsystem.getGyroscopeRotation().getDegrees(), HeadingTarget.GYRO);
    }

    @Override
    public void execute() {
        gyroRotation = m_drivetrainSubsystem.getGyroscopeRotation();

        rotationSpeed = m_rotationSupplier.getAsDouble();
    
        if (Math.abs(rotationSpeed) < 0.02) {
            // If we were manually rotating and have stopped, save this heading as our new target
            if (manuallyRotating) {
                m_drivetrainSubsystem.setTargetHeading(null); // Use this to disable heading control after rotating until a preset is pressed again
                previousHeadingType = null;
            }

            Double targetHeadingDegrees = m_drivetrainSubsystem.getTargetHeading();
            if (targetHeadingDegrees != null) {
                HeadingTarget headingType = m_drivetrainSubsystem.getTargetHeadingType();
                Rotation2d heading = headingType == HeadingTarget.POSE ? m_drivetrainSubsystem.getPose().getRotation() : gyroRotation;
                if (headingType != previousHeadingType) {
                    // Reset the PID controller if the heading type has changed
                    m_drivetrainSubsystem.resetHeadingPID(heading);
                }
                rotationSpeed = m_drivetrainSubsystem.calculateHeadingPID(heading, targetHeadingDegrees);
                previousHeadingType = headingType;
            }
            manuallyRotating = false;
        } else {
            manuallyRotating = true;
        }

        JoystickInputs inputs = m_joystickInputsSupplier.get();
        ChassisSpeeds speeds;
        if (m_drivetrainSubsystem.pigeonConnected()) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(inputs.x, inputs.y, rotationSpeed, gyroRotation);
        } else {
            speeds = new ChassisSpeeds(inputs.x, inputs.y, rotationSpeed);
        }

        applyCoralAimAssist(speeds, inputs);
        m_drivetrainSubsystem.drive(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        if (DriverStation.isAutonomous()) return;
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    public void applyCoralAimAssist(ChassisSpeeds speeds, JoystickInputs inputs) {
        Pose2d coralPose = RobotContainer.instance.vision.getCoralPose();
        if (coralPose == null || !RobotContainer.instance.driverController.leftTrigger().getAsBoolean()) {
            return;
        }

        Pose2d currentPose = m_drivetrainSubsystem.getPose();
        double xFeedback = coralPose.getX() - currentPose.getX();
        double yFeedback = coralPose.getY() - currentPose.getY();
        ChassisSpeeds coralSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xFeedback, yFeedback, 0, currentPose.getRotation());

        Translation2d coralTranslation = coralPose.getTranslation();
        Translation2d rayStart = m_drivetrainSubsystem.getPose().getTranslation();
        Translation2d rayTest = rayStart.plus(new Translation2d(inputs.y, inputs.x));

        // Ensure we're driving towards the coral
        // if (rayTest.getDistance(coralTranslation) <= rayStart.getDistance(coralTranslation)) {
        //     return;
        // }

        Translation2d rayEnd = rayStart.plus(new Translation2d(inputs.y * 1000, inputs.x * 1000));
    
        double perpdist = Helpers.perpendicularLineLength(coralTranslation, rayStart, rayEnd);
        perpdist = Math.pow(perpdist, coralAssistExponent);

        coralSpeeds.vxMetersPerSecond *= (perpdist * coralAssistKp);
        coralSpeeds.vyMetersPerSecond *= (perpdist * coralAssistKp);
        
        // Track the top speed of the speeds request, so that we can keep the coral assist from exceeding that
        double maxSpeed = Math.max(Math.abs(speeds.vxMetersPerSecond), Math.abs(speeds.vyMetersPerSecond));
        Helpers.addChassisSpeedsOverwrite(speeds, coralSpeeds);
        speeds.vxMetersPerSecond = Helpers.capValue(speeds.vxMetersPerSecond, maxSpeed);
        speeds.vyMetersPerSecond = Helpers.capValue(speeds.vyMetersPerSecond, maxSpeed);
    }
}
