package frc.robot.subsystems.elevatorArm;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.elevatorArm.ElevatorArmIO.ElevatorArmIOInputs;

@Logged
public class ElevatorArmSubsystem extends SubsystemBase{

    ElevatorArmIO io;
    ElevatorArmIOInputs inputs;

    public final Trigger hasCoral;
    public final Trigger hasPartialCoral;
    public final Trigger hasNoCoral;

    public ElevatorArmSubsystem() {
        inputs = new ElevatorArmIOInputs();
        io = new ElevatorArmIOSim();

        hasCoral = new Trigger(() -> inputs.middleCoralSensor);
        hasPartialCoral = new Trigger(() -> inputs.frontCoralSensor || inputs.middleCoralSensor || inputs.backCoralSensor);
        hasNoCoral = hasPartialCoral.negate();
    }

    @Override
    public void periodic() {
        io.update(inputs);
    }

    public Command runRollers() {
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

    public Command test() {
        return this.run(() -> {
          io.runMotorTest();
        });
    }   
}
