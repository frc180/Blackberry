package frc.robot.subsystems.elevatorArmAlgae;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevatorArmAlgae.ElevatorArmAlgaeIO.ElevatorArmAlgaeInputs;

@Logged
public class ElevatorArmAlgaeSubsystem extends SubsystemBase{

    private static final double FAR_OBJECT_THRESHOLD = 0.33;
    private static final double CLOSE_OBJECT_THRESHOLD = 0.17;

    ElevatorArmAlgaeIO io;
    ElevatorArmAlgaeInputs inputs;

    public Trigger hasAlgae;

    public Trigger farAlgae, closeAlgae;

    public ElevatorArmAlgaeSubsystem() {
        inputs = new 
        ElevatorArmAlgaeInputs();
        if (Robot.isReal()) {
            io = new ElevatorArmAlgaeIOTalonFX();
            // io = new ElevatorArmAlgaeIOSim();
        } else {
            io = new ElevatorArmAlgaeIOSim();
        }

        hasAlgae = new Trigger(() -> inputs.hasAlgae);
    }

    @Override
    public void periodic() {
        io.update(inputs);
    }

    // TODO: Minor fix - prevent this from running when doing left-aligned L2 and L3 (since we only get algae from the right side)
    public Command intakeBasedOnElevator() {
        ElevatorSubsystem elevator = RobotContainer.instance.elevator;
        return runEnd(
            () -> {
                if (elevator.isTargetingReefAlgaePosition()) {
                    io.setSpeed(0.5);
                } else {
                    io.setSpeed(0);
                }
            },
            () -> io.setSpeed(0)
        );
    }

    public Command pulseWhenNeeded() {
        return runEnd(
            () -> {
                if (inputs.distance > 0.05) {
                    io.setSpeed(0.05);
                } else {
                    io.setSpeed(0);
                }
            },
            () -> io.setSpeed(0)
        );
    }

    public Command holdPulse() {
        return runSpeed(0.05).withTimeout(0.5)
                .andThen(Commands.waitSeconds(0.3))
                .repeatedly();
    }

    public Command spit() {
        return runSpeed(-1);
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

    public Command stop() {
        return run(() -> io.stop());
    }
    
    public double getSpeed() {
        return inputs.speed;
    }
}
