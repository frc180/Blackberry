package frc.robot.subsystems.IntakeAlgae;

import static edu.wpi.first.units.Units.Inches;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeAlgaePivot.IntakeAlgaePivotSubsystem;
import frc.robot.util.simulation.SimLogic;

public class IntakeAlgaeIOSim implements IntakeAlgaeIO {

    double rollerSpeed = 0;
    IntakeSimulation intakeSim = null;

    public IntakeAlgaeIOSim() {
        if (RobotContainer.MAPLESIM) {
            intakeSim = IntakeSimulation.OverTheBumperIntake(
                "Algae",
                RobotContainer.instance.drivetrain.getDriveSim(),
                Inches.of(15),
                Inches.of(6),
                IntakeSide.LEFT,
                1
            );
        }
    }

    @Override   
    public void startRollers() {
        rollerSpeed = 1;
    }

    @Override
    public void stopRollers() {
        rollerSpeed = 0;
    }

    @Override
    public void update(IntakeAlgaeIOInputs inputs) {
        IntakeAlgaePivotSubsystem algaePivot = RobotContainer.instance.intakeAlgaePivot;
        boolean ableToIntake = algaePivot.getTargetDegrees() == algaePivot.extend && algaePivot.isAtTarget() && rollerSpeed > 0;

        if (intakeSim != null) {
            // Using physics simulation with simulated intake
            if (ableToIntake) {
                intakeSim.startIntake();
            } else {
                intakeSim.stopIntake();
            }
            if (!SimLogic.intakeHasAlgae) {
                SimLogic.intakeHasAlgae = intakeSim.obtainGamePieceFromIntake();
            }
        } else {
            // No physics sim, assume there's an algae if we are intaking
            if (!SimLogic.intakeHasAlgae) {
                SimLogic.intakeHasAlgae = ableToIntake;
            }
        }
        inputs.hasAlgae = SimLogic.intakeHasAlgae;
    }
}
