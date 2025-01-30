package frc.robot.subsystems.elevatorArmPivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.elevatorArmPivot.ElevatorArmPivotIO;
import frc.robot.subsystems.elevatorArmPivot.ElevatorArmPivotIO.ElevatorArmPivotIOInputs;

public class ElevatorArmPivotSubsystem extends SubsystemBase{

    private ElevatorArmPivotIO io;
    private ElevatorArmPivotIOInputs inputs;

    public final double receiving = -45;
    public final double horizontal = 0;
    public final double score = 45;
    public final double L1Score = 60;

    public double targetPosition = 0;

    public Trigger elevatorArmInPosition = new Trigger(() -> isElevatorArmInPosition());

    public ElevatorArmPivotSubsystem() {
        inputs = new ElevatorArmPivotIOInputs();
        io = new ElevatorArmPivotIOSim();
    }

    @Override
    public void periodic() {
        io.update(inputs);
    }

    public Command receivePosition() {
        return this.run(() -> {
            io.setPosition(receiving);
        });
    }

    public Command stowPosition() {
        return this.run (() -> {
            io.setPosition(horizontal);
        });
    }

    public Command scorePosition() {
        return this.run(() -> {
            io.setPosition(score);
        });
    }

    public Command lowScorePosition() {
        return this.run (() -> {
            io.setPosition(L1Score);
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


    
}
