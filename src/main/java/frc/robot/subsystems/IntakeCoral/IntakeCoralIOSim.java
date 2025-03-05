package frc.robot.subsystems.IntakeCoral;

import static edu.wpi.first.units.Units.Inches;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeCoralPivot.IntakeCoralPivotSubsystem;
import frc.robot.util.simulation.SimLogic;

public class IntakeCoralIOSim implements IntakeCoralIO {
    
    private IntakeSimulation intakeSim = null;

    double rollerSpeed = 0;

    public IntakeCoralIOSim() {
        if (RobotContainer.MAPLESIM) {
            intakeSim = IntakeSimulation.OverTheBumperIntake(
                "Coral",
                RobotContainer.instance.drivetrain.getDriveSim(),
                Inches.of(15),
                Inches.of(6),
                IntakeSide.BACK,
                1
            );
        }
    }

    @Override
    public void update(IntakeIOInputs inputs) {
        IntakeCoralPivotSubsystem coralIntake = RobotContainer.instance.intakeCoralPivot;
        boolean intakeDeployed = coralIntake.getTargetPosition() == IntakeCoralPivotSubsystem.extend && coralIntake.isAtTarget();
        boolean intakeReady = intakeDeployed && rollerSpeed > 0;

        if (intakeSim != null) {
            if (intakeReady) {
                intakeSim.startIntake();
            } else {
                intakeSim.stopIntake();
            }
            if (!SimLogic.intakeHasCoral) {
                SimLogic.intakeHasCoral = intakeSim.obtainGamePieceFromIntake();
            }
        } else {
            Pose2d coralPose = RobotContainer.instance.vision.getCoralPose();
            if (coralPose != null && rollerSpeed != 0) {
                if (SimLogic.intakeHasCoral && rollerSpeed < 0) {
                    // We have the coral and are ejecting it
                    SimLogic.intakeHasCoral = false;
                } else if (!SimLogic.intakeHasCoral && intakeReady) {
                    // Say we have the coral if we are within 0.75 meters of it
                    Pose2d robotPose = RobotContainer.instance.drivetrain.getPose();
                    double distance = robotPose.getTranslation().getDistance(coralPose.getTranslation());
                    SimLogic.intakeHasCoral = distance <= 0.75;
                }
            }
        }

        inputs.voltage = rollerSpeed * 12;
        inputs.coralSensor = SimLogic.intakeHasCoral;
    }

    @Override
    public void setSpeed(double speed) {
        rollerSpeed = speed;
        SmartDashboard.putNumber("DEBUG Coral Requested Speed", speed);
    }
}
