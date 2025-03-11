package frc.robot.subsystems.elevatorArmAlgae;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevatorArmAlgae.ElevatorArmAlgaeIO.ElevatorArmAlgaeInputs;

@Logged
public class ElevatorArmAlgaeSubsystem extends SubsystemBase {

    protected static final double FAR_OBJECT_THRESHOLD = 0.28; // was 0.23 which is probably too low
    protected static final double CLOSE_OBJECT_THRESHOLD = 0.2;
    protected static final double HAS_ALGAE_THRESHOLD = 0.1;

    ElevatorArmAlgaeIO io;
    ElevatorArmAlgaeInputs inputs;

    private final LinearFilter distanceFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
    private double distanceFiltered = 0;

    public final Trigger hasAlgae;
    public final Trigger farAlgae, closeAlgae, closeAlgaeDebounced;


    public ElevatorArmAlgaeSubsystem() {
        inputs = new ElevatorArmAlgaeInputs();
        if (Robot.isReal()) {
            io = new ElevatorArmAlgaeIOTalonFX();
            // io = new ElevatorArmAlgaeIOSim();
        } else {
            io = new ElevatorArmAlgaeIOTalonFX();
            // io = new ElevatorArmAlgaeIOSim();
        }

        farAlgae = new Trigger(() -> distanceFiltered > FAR_OBJECT_THRESHOLD);
        closeAlgae = new Trigger(() -> distanceFiltered < CLOSE_OBJECT_THRESHOLD);
        hasAlgae = new Trigger(() -> distanceFiltered < HAS_ALGAE_THRESHOLD);

        closeAlgaeDebounced = closeAlgae.debounce(1, DebounceType.kFalling);
    }

    @Override
    public void periodic() {
        io.update(inputs);
        distanceFiltered = inputs.distance;
        // distanceFiltered = distanceFilter.calculate(inputs.distance);
    }
    
    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
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

    public Command intakeAndIndex(double speed) {
        return indexWithIdle(speed, speed);
    }

    public Command passiveIndex() {
        return indexWithIdle(0, 1);
    }

    private Command indexWithIdle(double idleSpeed, double indexSpeed) {
        return runEnd(
            () -> {
                if (hasAlgae.getAsBoolean()) {
                    io.setSpeed(0.05);
                } else if (closeAlgaeDebounced.getAsBoolean() ) {
                    io.setSpeed(indexSpeed);
                } else {
                    io.setSpeed(idleSpeed);
                }
            },
            () -> io.setSpeed(0.0)
        );
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
