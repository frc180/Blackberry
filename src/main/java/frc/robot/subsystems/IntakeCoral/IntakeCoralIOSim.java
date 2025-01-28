package frc.robot.subsystems.IntakeCoral;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeCoralPivot.IntakeCoralPivotSubsystem;

public class IntakeCoralIOSim implements IntakeCoralIO {
    
    double rollerSpeed = 0;
    boolean hasCoral = false;

    public IntakeCoralIOSim() {}

    @Override
    public void update(IntakeIOInputs inputs) {
        Pose2d coralPose = RobotContainer.instance.vision.getCoralPose();
        if (coralPose != null && rollerSpeed != 0) {
            if (hasCoral && rollerSpeed < 0) {
                // We have the coral and are ejecting it
                hasCoral = false;
            } else if (!hasCoral && rollerSpeed > 0) {
                IntakeCoralPivotSubsystem coralIntake = RobotContainer.instance.intakeCoralPivot;
                if (coralIntake.getTargetDegrees() == IntakeCoralPivotSubsystem.extend && coralIntake.isAtTarget()) {
                    // Say we have the coral if we are within 0.75 meters of it
                    Pose2d robotPose = RobotContainer.instance.drivetrain.getPose();
                    double distance = robotPose.getTranslation().getDistance(coralPose.getTranslation());
                    hasCoral = distance <= 0.75;
                }
            }
        }

        inputs.voltage = rollerSpeed * 12;
        inputs.coralSensor = hasCoral;
    }

    @Override
    public void startRollers() {
        rollerSpeed = 1;
        // System.out.println("Coral Intake Roller Speed: " + rollerSpeed);
    }

    @Override
    public void stopRollers() {
        rollerSpeed = 0;
        // System.out.println("Coral Intake Roller Speed: " + rollerSpeed);
    }
}
