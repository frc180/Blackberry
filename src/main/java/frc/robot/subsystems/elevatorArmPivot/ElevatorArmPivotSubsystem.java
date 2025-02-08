package frc.robot.subsystems.elevatorArmPivot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.elevatorArmPivot.ElevatorArmPivotIO;
import frc.robot.subsystems.elevatorArmPivot.ElevatorArmPivotIO.ElevatorArmPivotIOInputs;
import frc.robot.util.simulation.SimVisuals;

@Logged
public class ElevatorArmPivotSubsystem extends SubsystemBase{

    private ElevatorArmPivotIO io;
    private ElevatorArmPivotIOInputs inputs;

    public static final double receiving = 45;
    public static final double algaeReceive = -70;
    public static final double horizontal = 0;
    public static final double score = 0;
    public static final double L1Score = -60;
    public static final double netPosition = -80;

    public double targetPosition = 0;

    public Trigger elevatorArmInPosition = new Trigger(() -> isElevatorArmInPosition());
    public Trigger elevatorArmInScoringPosition = new Trigger (() -> isElevatorArmInScoringPosition());

    public ElevatorArmPivotSubsystem() {
        inputs = new ElevatorArmPivotIOInputs();
        io = new ElevatorArmPivotIOSim();
    }

    @Override
    public void periodic() {
        io.update(inputs);
        SimVisuals.setElevatorArmPivotDegrees(inputs.position);
    }

    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public Command receivePosition() {
        return this.run(() -> {
            io.setPosition(receiving);
            targetPosition = receiving;
        });
    }

    public Command stowPosition() {
        return this.run (() -> {
            io.setPosition(horizontal);
            targetPosition = horizontal;

        });
    }

    public Command scorePosition() {
        return this.run(() -> {
            io.setPosition(score);
            targetPosition = score;

        });
    }

    public Command lowScorePosition() {
        return this.run (() -> {
            io.setPosition(L1Score);
            targetPosition = L1Score;

        });
    }

    public Command netScorePosition() {
        return this.run (() -> {
            io.setPosition(netPosition);
            targetPosition = netPosition;
        });
    }

    public Command receiveAlgaePosition() {
        return this.run (() -> {
            io.setPosition(algaeReceive);
            targetPosition = algaeReceive;
        });
    }

    public boolean isElevatorArmInPosition() {
        if (Math.abs(targetPosition - inputs.position) <= 0.025) {
            return true;
        } else {
            return false;
        }
    }

    public void setArmPositionDirect(double position) {
        io.setPosition(position);
        targetPosition = position;
    }

    public boolean isAtReceivingPosition() {
        return (targetPosition == receiving || targetPosition == algaeReceive) && isElevatorArmInPosition();
    }

    public boolean isElevatorArmInScoringPosition() {
        return isElevatorArmInPosition() && (targetPosition == score || targetPosition == L1Score || targetPosition == netPosition);
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
    
}
