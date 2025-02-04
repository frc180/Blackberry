package frc.robot.subsystems.elevatorArmAlgae;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.elevatorArmAlgae.ElevatorArmAlgaeIO.ElevatorArmAlgaeInputs;

@Logged
public class ElevatorArmAlgaeSubsystem extends SubsystemBase{

    ElevatorArmAlgaeIO io;
    ElevatorArmAlgaeInputs inputs;

    public Trigger hasAlgae;

    public ElevatorArmAlgaeSubsystem() {
        inputs = new 
        ElevatorArmAlgaeInputs();
        if (Robot.isReal()) {
            io = new ElevatorArmAlgaeIOTalonFX();
        } else {
            io = new ElevatorArmAlgaeIOSim();
        }

        hasAlgae = new Trigger(() -> inputs.hasAlgae);
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
