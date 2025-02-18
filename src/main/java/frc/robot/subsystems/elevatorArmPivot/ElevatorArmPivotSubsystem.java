package frc.robot.subsystems.elevatorArmPivot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.RobotCentric;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevatorArmPivot.ElevatorArmPivotIO.ElevatorArmPivotIOInputs;
import frc.robot.util.simulation.SimVisuals;

@Logged
public class ElevatorArmPivotSubsystem extends SubsystemBase{

    private ElevatorArmPivotIO io;
    private ElevatorArmPivotIOInputs inputs;

    // TODO: read manually from robot
    protected static final Angle FORWARD_LIMIT = Degrees.of(100);
    protected static final Angle REVERSE_LIMIT = Degrees.of(-75);
    protected static final Angle HARD_STOP_OFFSET = Degrees.of(25);

    public static final double L4_SCORE = Units.degreesToRotations(-14);
    public static final double L3_SCORE = Units.degreesToRotations(-3);
    public static final double L2_SCORE = L3_SCORE;
    public static final double L1_SCORE = Units.degreesToRotations(0);

    public static final double receiving = Units.degreesToRotations(45);
    public static final double algaeReceive = Units.degreesToRotations(-70);
    public static final double horizontal = 0;
    public static final double score = 0;
    public static final double L1Score = Units.degreesToRotations(-60);
    public static final double netScore = Units.degreesToRotations(80);
    public static final double netScoreBackwards = Units.degreesToRotations(100);

    private static final double IN_POSITION_TOLERANCE = Units.degreesToRotations(2);

    private Double targetPositionMax = null;
    private Double targetPositionMin = null;

    private double targetPosition = 0;
    private boolean pidMode = false;

    @NotLogged
    public Trigger elevatorArmInPosition = new Trigger(() -> isElevatorArmInPosition());
    @NotLogged
    public Trigger elevatorArmInScoringPosition = new Trigger (() -> isElevatorArmInScoringPosition());
    public Trigger stalling = new Trigger(this::isStalling).debounce(1);

    public ElevatorArmPivotSubsystem() {
        inputs = new ElevatorArmPivotIOInputs();
        if (Robot.isReal()) {
            // io = new ElevatorArmPivotIOSim();
            io = new ElevatorArmPivotIOTalonFX();
        } else {
            io = new ElevatorArmPivotIOTalonFX();
        }
    }

    @Override
    public void periodic() {
        io.update(inputs);
        SimVisuals.setElevatorArmPivotDegrees(getDegrees());

        if (pidMode) {
            setArmPositionDirect(targetPosition);
        }
    }

    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public Command receivePosition() {
        return setPosition(receiving);
    }

    public Command stowPosition() {
        return setPosition(horizontal);
    }

    public Command scorePosition() {
        return setPosition(score);
    }

    public Command lowScorePosition() {
        return setPosition(L1Score);
    }

    public Command netScorePosition() {
        return setPosition(netScore);
    }

    public Command netScoreBackwardsPosition() {
        return setPosition(netScoreBackwards);
    }

    public Command receiveAlgaePosition() {
        return setPosition(algaeReceive);
    }

    public Command home() {
        return runSpeed(-0.1).until(stalling)
                .andThen(zero(HARD_STOP_OFFSET));
    }

    public Command zero(Angle angle) {
        return zero(angle.in(Rotations));
    }

    public Command zero(double rotations) {
        return runOnce(() -> io.zero(rotations));
    }

    // untested
    public Command matchElevatorPreset() {
        ElevatorSubsystem elevator = RobotContainer.instance.elevator;
        return run(() -> {
            if (elevator.getTargetPosition() == ElevatorSubsystem.L4) {
                setArmPositionDirect(L4_SCORE);
            } else if (elevator.getTargetPosition() == ElevatorSubsystem.L3) {
                setArmPositionDirect(L3_SCORE);
            } else if (elevator.getTargetPosition() == ElevatorSubsystem.L2) {
                setArmPositionDirect(L2_SCORE);
            } else if (elevator.getTargetPosition() == ElevatorSubsystem.L1) {
                setArmPositionDirect(L1_SCORE);
            } else {
                setArmPositionDirect(receiving);
            }
        });
    }

    public Command setPosition(double position) {
        return run(() -> setArmPositionDirect(position));
    }

    public Command setSpeed(double speed) {
        return run(() -> {
            io.setSpeed(speed);
            pidMode = false;
        });
    }

    public Command runSpeed(double speed) {
        return runEnd(
            () -> {
                io.setSpeed(speed);
                pidMode = false;
            },
            () -> io.stopMotor()
        );
    }

    public Command stop() {
        return run(() -> {
            io.stopMotor();
            pidMode = false;
        });
    }

    public void setPositionLimits(Double min, Double max) {
        targetPositionMin = min;
        targetPositionMax = max;
    }

    public void setArmPositionDirect(double position) {
        targetPosition = position;
        pidMode = true;
        if (targetPositionMin != null) position = Math.max(position, targetPositionMin);
        if (targetPositionMax != null) position = Math.min(position, targetPositionMax);
        io.setPosition(position);
    }

    public boolean isElevatorArmInPosition() {
        return Math.abs(targetPosition - inputs.position) <= IN_POSITION_TOLERANCE;
    }

    public boolean isAtReceivingPosition() {
        if (!isElevatorArmInPosition()) return false;
        
        return (targetPosition == receiving || targetPosition == algaeReceive);
    }

    public boolean isElevatorArmInScoringPosition() {
        if (!isElevatorArmInPosition()) return false;

        return targetPosition == score || targetPosition == L1Score || targetPosition == netScore || targetPosition == netScoreBackwards;
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

    public boolean isStalling() {
        return Math.abs(inputs.current) >= 40 && Math.abs(inputs.velocity) <= 0.01;
    }
}
