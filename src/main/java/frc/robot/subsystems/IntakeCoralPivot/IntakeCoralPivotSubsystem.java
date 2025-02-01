package frc.robot.subsystems.IntakeCoralPivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeCoralPivot.IntakeCoralPivotIO.IntakeCoralPivotIOInputs;
import frc.robot.util.simulation.SimVisuals;

public class IntakeCoralPivotSubsystem extends SubsystemBase {
    
    //presets for intake positions
    public static final double stow = 90;
    public static final double extend = 200;

    public IntakeCoralPivotIO io;
    private IntakeCoralPivotIOInputs inputs;

    private double targetPosition = 0;

    public final Trigger atTarget = new Trigger(this::isAtTarget);

    public IntakeCoralPivotSubsystem() {
        if (Robot.isReal()) {
            io = new IntakeCoralPivotIOTalonFXS();
        } else {
            io = new IntakeCoralPivotIOSim();
        }

        inputs = new IntakeCoralPivotIOInputs();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.update(inputs);
        SimVisuals.setCoralIntakeDegrees(inputs.position);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation.
        // Runs before periodic()
        io.simulationPeriodic();
    }

    public boolean isAtTarget() {
        return Math.abs(inputs.position - targetPosition) <= 6;
    }

    public double getTargetDegrees() {
        return targetPosition;
    }

    public Command setPosition(double encoderPosition) {
        return this.run(() -> {
            io.setIntakePosition(encoderPosition);
            targetPosition = encoderPosition;
          });
    }
}
