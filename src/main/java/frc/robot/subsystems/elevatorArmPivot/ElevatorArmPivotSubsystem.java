package frc.robot.subsystems.elevatorArmPivot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.elevatorArmPivot.ElevatorArmPivotIO.ElevatorArmPivotIOInputs;
import frc.robot.util.simulation.SimVisuals;

@Logged
public class ElevatorArmPivotSubsystem extends SubsystemBase{

    private ElevatorArmPivotIO io;
    private ElevatorArmPivotIOInputs inputs;

    public static final double receiving = Units.degreesToRotations(45);
    public static final double algaeReceive = Units.degreesToRotations(-70);
    public static final double horizontal = 0;
    public static final double score = 0;
    public static final double L1Score = Units.degreesToRotations(-60);
    public static final double netScore = Units.degreesToRotations(80);
    public static final double netScoreBackwards = Units.degreesToRotations(100);

    private static final double IN_POSITION_TOLERANCE = Units.degreesToRotations(2);

    private double targetPosition = 0;

    @NotLogged
    public Trigger elevatorArmInPosition = new Trigger(() -> isElevatorArmInPosition());
    @NotLogged
    public Trigger elevatorArmInScoringPosition = new Trigger (() -> isElevatorArmInScoringPosition());

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

    public Command setPosition(double position) {
        return run(() -> setArmPositionDirect(position));
    }

    public void setArmPositionDirect(double position) {
        io.setPosition(position);
        targetPosition = position;
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

    public Command setSpeed(double speed) {
        return this.run(() -> {
          io.setSpeed(speed);
        });
    }

    public Command stop() {
        return this.run(() -> {
            io.stopMotor();
        });
    }
    
}
