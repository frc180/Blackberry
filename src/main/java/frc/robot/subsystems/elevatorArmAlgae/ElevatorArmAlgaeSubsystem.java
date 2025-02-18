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
            // io = new ElevatorArmAlgaeIOTalonFX();
            io = new ElevatorArmAlgaeIOSim();
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

    public Command spit() {
        return this.runEnd(() -> {
            io.reverse();
        }, () -> {
            io.stop();
        });
    }

    public void reverseDirect() {
        io.reverse();
    }

    public Command setSpeed(double speed) {
        return run(() -> io.setSpeed(speed));
    }

    public Command runSpeed(double speed) {
        return runEnd(
            () -> io.setSpeed(speed),
            () -> io.stop()
        );
    }
    
    public double getSpeed() {
        return inputs.speed;
    }
}
