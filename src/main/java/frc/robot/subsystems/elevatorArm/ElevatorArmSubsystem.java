package frc.robot.subsystems.elevatorArm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.elevatorArm.ElevatorArmIO.ElevatorArmIOInputs;

public class ElevatorArmSubsystem extends SubsystemBase{

    ElevatorArmIO io;
    ElevatorArmIOInputs inputs;

    public ElevatorArmSubsystem() {
        inputs = new ElevatorArmIOInputs();
        io = new ElevatorArmIOSim();
    }

    @Override
    public void periodic() {
        io.update(inputs);
    }

    public Command runArm() {
        return this.run(() -> {
            io.run();
        });
    }

    public Command reverseRun() {
        return this.run (() -> {
            io.reverse();
        });
    }

    public Command stop() {
        return this.run(() -> {
            io.stop();
        });
    }


    
}
