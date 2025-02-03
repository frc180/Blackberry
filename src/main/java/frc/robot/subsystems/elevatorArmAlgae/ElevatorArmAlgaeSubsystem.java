package frc.robot.subsystems.elevatorArmAlgae;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.elevatorArmAlgae.ElevatorArmAlgaeIO.ElevatorArmAlgaeInputs;

public class ElevatorArmAlgaeSubsystem extends SubsystemBase{

    ElevatorArmAlgaeIO io;
    ElevatorArmAlgaeInputs inputs;

    public ElevatorArmAlgaeSubsystem() {
        if (Robot.isReal()) {
            io = new ElevatorArmAlgaeIOTalonFX();
        } else {
            io = new ElevatorArmAlgaeIOSim();
        }
    }

    @Override
    public void periodic() {
        io.update(inputs);
    }

    public Command run() {
        return this.run(() -> {
            io.run();
        });
    }

    public Command stop() {
        return this.run (() -> {
            io.stop();
        });
    }

    public Command reverse() {
        return this.run(() -> {
            io.reverse();
        });
    }
    
}
