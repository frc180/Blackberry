package frc.robot.subsystems.IntakeCoral;

import static edu.wpi.first.units.Units.Inches;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import edu.wpi.first.math.geometry.Pose2d;
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
                Inches.of(12),
                IntakeSide.BACK,
                1
            );
        }
    }

    @Override
    public void update(IntakeIOInputs inputs) {
        IntakeCoralPivotSubsystem coralIntake = RobotContainer.instance.intakeCoralPivot;
        boolean intakeDeployed = coralIntake.getTargetDegrees() == IntakeCoralPivotSubsystem.extend && coralIntake.isAtTarget();
        boolean intakeReady = intakeDeployed && rollerSpeed > 0;

        if (intakeSim != null) {
            if (intakeReady) {
                intakeSim.startIntake();
            } else {
                intakeSim.stopIntake();
            }
            if (!SimLogic.hasCoral) {
                SimLogic.hasCoral = intakeSim.obtainGamePieceFromIntake();
            }
        } else {
            Pose2d coralPose = RobotContainer.instance.vision.getCoralPose();
            if (coralPose != null && rollerSpeed != 0) {
                if (SimLogic.hasCoral && rollerSpeed < 0) {
                    // We have the coral and are ejecting it
                    SimLogic.hasCoral = false;
                } else if (!SimLogic.hasCoral && intakeReady) {
                    // Say we have the coral if we are within 0.75 meters of it
                    Pose2d robotPose = RobotContainer.instance.drivetrain.getPose();
                    double distance = robotPose.getTranslation().getDistance(coralPose.getTranslation());
                    SimLogic.hasCoral = distance <= 0.75;
                }
            }
        }

        inputs.voltage = rollerSpeed * 12;
        inputs.coralSensor = SimLogic.hasCoral;
    }

    @Override
    public void startRollers() {
        rollerSpeed = 1;
    }

    @Override
    public void stopRollers() {
        rollerSpeed = 0;
    }
}
