package frc.robot.subsystems.IntakeCoralPivot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeCoralPivot.IntakeCoralPivotIO.IntakeCoralPivotIOInputs;
import frc.robot.util.simulation.SimVisuals;

@Logged
public class IntakeCoralPivotSubsystem extends SubsystemBase {
    
    //presets for intake positions
    public static final double stow = Units.degreesToRotations(90);
    public static final double extend = Units.degreesToRotations(200);

    private static final double IN_POSITION_TOLERANCE = Units.degreesToRotations(6);

    public IntakeCoralPivotIO io;
    private IntakeCoralPivotIOInputs inputs;

    private double targetPosition = 0;

    @NotLogged
    public final Trigger atTarget = new Trigger(this::isAtTarget);
    @NotLogged
    public final Trigger atStowPosition = new Trigger(this::isAtStowPosition);

    public IntakeCoralPivotSubsystem() {
        if (Robot.isReal()) {
            io = new IntakeCoralPivotIOSim();
        } else {
            io = new IntakeCoralPivotIOTalonFXS();
        }

        inputs = new IntakeCoralPivotIOInputs();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.update(inputs);
        SimVisuals.setCoralIntakeDegrees(getDegrees());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation.
        // Runs before periodic()
        io.simulationPeriodic();
    }

    public boolean isAtTarget() {
        return Math.abs(inputs.position - targetPosition) <= IN_POSITION_TOLERANCE;
    }

    @NotLogged
    public double getTargetPosition() {
        return targetPosition;
    }

    public double getDegrees() {
        return Units.rotationsToDegrees(inputs.position);
    }

    public double getTargetDegrees() {
        return Units.rotationsToDegrees(targetPosition);
    }

    public Command setPosition(double encoderPosition) {
        return this.run(() -> {
            io.setIntakePosition(encoderPosition);
            targetPosition = encoderPosition;
          });
    }

    public Command test() {
        return this.run(() -> {
          io.runMotorTest();
        });
    }

    public Command stop() {
        return this.run(() -> {
            io.stopMotor();
        });
    }

    public boolean isAtStowPosition() {
        return targetPosition == stow && isAtTarget();
    }
}
