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
        // TODO: depending on how coral indexing works on the real robot, "hasCoral" might need to require all sensors (or 2/3)
        // hasCoral = new Trigger(() -> inputs.frontCoralSensor && inputs.middleCoralSensor && inputs.backCoralSensor);
        hasPartialCoral = new Trigger(() -> inputs.frontCoralSensor || inputs.middleCoralSensor || inputs.backCoralSensor);
        hasNoCoral = hasPartialCoral.negate();
    }

    @Override
    public void periodic() {
        io.update(inputs);
    }

    public Command intakeAndIndex() {
        return run(() -> {
            // All the sensors are triggered means coral is perfectly positioned
            if (inputs.frontCoralSensor && inputs.middleCoralSensor && inputs.backCoralSensor) {
                io.setSpeed(0);
                return;
            }

            // Middle or back sensor without front sensor means we overshot
            if (!inputs.frontCoralSensor && (inputs.middleCoralSensor || inputs.backCoralSensor)) {
                io.setSpeed(-0.25);
                return;
            }

            // Middle sensor without back sensor means we need to move forward still
            if (inputs.middleCoralSensor && !inputs.backCoralSensor) {
                io.setSpeed(0.25);
                return;
            }

            io.setSpeed(1);
        });
    }

    public Command runRollers() {
        return run(() -> io.run());
    }

    public Command reverseRun() {
        return run(() -> io.reverse());
    }

    public Command stop() {
        return run(() -> io.stop());
    }

    public Command test() {
        return run(() -> io.runMotorTest());
    }   
}
