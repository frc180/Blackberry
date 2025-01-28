package frc.robot.subsystems.IntakeCoralPivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeCoralPivot.IntakeCoralPivotIO;
import frc.robot.subsystems.IntakeCoralPivot.IntakeCoralPivotIO.IntakeCoralPivotIOInputs;
import frc.robot.subsystems.IntakeCoralPivot.IntakeCoralPivotIOSim;
import frc.robot.subsystems.IntakeCoralPivot.IntakeCoralPivotIOTalonFXS;

public class IntakeCoralPivotSubsystem extends SubsystemBase {
    
    //presets for intake positions
    public static final double stow = 0;
    public static final double extend = 100;

    public IntakeCoralPivotIO io;
    private IntakeCoralPivotIOInputs inputs;

    private double targetPosition = 0;

    public IntakeCoralPivotSubsystem() {
        if (Robot.isReal()) {
            io = new IntakeCoralPivotIOSim();
        } else {
            io = new IntakeCoralPivotIOSim();
        }

        inputs = new IntakeCoralPivotIOInputs();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.update(inputs);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation.
        // Runs before periodic()
        io.simulationPeriodic();
    }

    public Command setPosition(double encoderPosition) {
        return this.run(() -> {
            io.setIntakePosition(encoderPosition);
            targetPosition = encoderPosition;
          });
    }
}
