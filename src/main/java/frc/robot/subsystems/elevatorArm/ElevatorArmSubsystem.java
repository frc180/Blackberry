package frc.robot.subsystems.elevatorArm;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
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
        if (Robot.isReal()) {
            io = new ElevatorArmIOTalonFX();
            // io = new ElevatorArmIOSim();
        } else {
            io = new ElevatorArmIOSim();
        }

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
        final double highLoad = 0.3;
        final double slowLoad = 0.05;
        return runEnd(() -> {
            // Middle sensor only means centered
            if (!inputs.backCoralSensor && inputs.middleCoralSensor && !inputs.frontCoralSensor) {
                io.setSpeed(0);
                return;
            }

            // Front sensor means we've overshot
            if (inputs.frontCoralSensor) {
                io.setSpeed(-slowLoad);
                return;
            }

            if (inputs.backCoralSensor && inputs.middleCoralSensor) {
                io.setSpeed(slowLoad);
                return;
            }

            io.setSpeed(highLoad);
        },
        () -> io.setSpeed(0));
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

    public Command setSpeed(double speed) {
        return run(() -> io.setSpeed(speed));
    }

    public Command runSpeed(double speed) {
        return runEnd(
            () -> io.setSpeed(speed),
            () -> io.stop()
        );
    }
}
