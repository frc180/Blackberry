package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.HeadingTarget;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.spamrobotics.util.JoystickInputs;

public class DefaultDriveCommand extends Command {
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
        m_drivetrainSubsystem.resetHeadingPID(HeadingTarget.GYRO);
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
        m_drivetrainSubsystem.drive(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        if (DriverStation.isAutonomous()) return;
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
