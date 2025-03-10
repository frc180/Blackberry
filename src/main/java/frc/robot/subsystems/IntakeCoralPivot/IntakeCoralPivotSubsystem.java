package frc.robot.subsystems.IntakeCoralPivot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeCoralPivot.IntakeCoralPivotIO.IntakeCoralPivotIOInputs;
import frc.robot.util.simulation.SimVisuals;

@Logged
public class IntakeCoralPivotSubsystem extends SubsystemBase {

    // enum CoralPivotState {
    //     SLAMMING,
    //     HOLD,
    //     MANUAL
    // }
    
    //presets for intake positions
    public static final double stow = Units.degreesToRotations(90);
    public static final double extend = Units.degreesToRotations(0);

    private static final double IN_POSITION_TOLERANCE = Units.degreesToRotations(3);

    private final IntakeCoralPivotIO io;
    private final IntakeCoralPivotIOInputs inputs;

    private final PIDController rioPID;

    // private CoralPivotState state = CoralPivotState.MANUAL;
    private double targetPosition = -1;
    private Trigger isStallingDebounce = new Trigger(this::isStalling).debounce(0.1);

    @NotLogged
    public final Trigger atTarget = new Trigger(this::isAtTarget);
    @NotLogged
    public final Trigger atStowPosition = new Trigger(this::isAtStowPosition);

    public IntakeCoralPivotSubsystem() {
        inputs = new IntakeCoralPivotIOInputs();
        if (Robot.isReal()) {
            io = new IntakeCoralPivotIOTalonFXS();
            // io = new IntakeCoralPivotIOSim();
        } else {
            io = new IntakeCoralPivotIOTalonFXS();
        }

        // Unused for now, may be utilized to PID off the string potentiometer
        rioPID = new PIDController(5, 0, 0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.update(inputs);
        SimVisuals.setCoralIntakeDegrees(180 - getDegrees());

        // double output = rioPID.calculate(inputs.position, targetPosition);
        // io.setSpeed(output);

        // if (state == CoralPivotState.SLAMMING && isStallingDebounce.getAsBoolean()) {
        //     state = CoralPivotState.HOLD;
        // }

        // if (state == CoralPivotState.SLAMMING) {
        //     double dir = targetPosition == stow ? -0.3 : 0.3;
        //     io.setSpeed(dir * 1);
        // } else if (state == CoralPivotState.HOLD) {
        //     io.setSpeed(0);
        // }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation.
        // Runs before periodic()
        io.simulationPeriodic();
    }

    public boolean isAtTarget() {
        // return state == CoralPivotState.HOLD;
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

    public Command stow() {
        return setPosition(stow);
    }

    public Command extend() {
        return setPosition(extend);
    }

    public Command setPosition(double encoderPosition) {
        return run(() -> {
            // if (targetPosition != encoderPosition) {
            //     state = CoralPivotState.SLAMMING;
            // } else if (state == CoralPivotState.MANUAL) {
            //     state = CoralPivotState.HOLD;
            // }
            targetPosition = encoderPosition;
            io.setIntakePosition(encoderPosition);
          });
    }

    public Command runSpeed(double speed) {
        return runEnd(
            () -> setSpeed(speed),
            () -> setSpeed(0)
        );
    }

    public Command stop() {
        return run(() -> {
            io.stopMotor();
            // state = CoralPivotState.MANUAL;
        });
    }

    public Command zero() {
        return zero(0);
    }

    public Command zero(Angle angle) {
        return zero(angle.in(Rotations));
    }

    public Command zero(double rotations) {
        return Commands.runOnce(() -> io.zero(rotations));
    }

    public void setSpeed(double speed) {
        io.setSpeed(speed);
        // state = CoralPivotState.MANUAL;
    }

    public boolean isAtStowPosition() {
        return targetPosition == stow && isAtTarget();
    }

    public boolean isStalling() {
        // return state == CoralPivotState.SLAMMING && Math.abs(inputs.velocity) < 0.001;
        return Math.abs(inputs.velocity) < 0.001;
    }
}
