package frc.robot.subsystems.elevatorArm;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
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

    @NotLogged
    final double slowIndexSpeed = 0.05;

    public ElevatorArmSubsystem() {
        inputs = new ElevatorArmIOInputs();
        if (Robot.isReal()) {
            io = new ElevatorArmIOTalonFX();
            // io = new ElevatorArmIOSim();
        } else {
            io = new ElevatorArmIOSim();
        }

        hasCoral = new Trigger(() -> inputs.middleCoralSensor);
        hasPartialCoral = new Trigger(() -> inputs.frontCoralSensor || inputs.middleCoralSensor || inputs.backCoralSensor);
        hasNoCoral = hasPartialCoral.negate();
    }

    @Override
    public void periodic() {
        io.update(inputs);
    }

    public Command intakeAndIndex() {
        return indexCommand(0.3);
    }

    public Command passiveIndex() {
        return indexCommand(0);
    }

    private Command indexCommand(double idleSpeed) {
        return runEnd(() -> {
            // Middle sensor only means we're centered, so stop
            if (!inputs.backCoralSensor && inputs.middleCoralSensor && !inputs.frontCoralSensor) {
                io.setSpeed(0);
                return;
            }

            // Front sensor means we've overshot, so reverse
            if (inputs.frontCoralSensor) {
                io.setSpeed(-slowIndexSpeed);
                return;
            }

            // Back and middle sensors means we're approaching the sweet spot, so move forward slowly
            if (inputs.backCoralSensor && inputs.middleCoralSensor) {
                io.setSpeed(slowIndexSpeed);
                return;
            }

            io.setSpeed(idleSpeed);
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
