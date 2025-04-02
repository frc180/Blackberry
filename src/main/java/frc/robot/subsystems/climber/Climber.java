package frc.robot.subsystems.climber;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.climber.ClimberIO.ClimberInputs;
import frc.robot.commands.RumbleCommand;

@Logged
public class Climber extends SubsystemBase {

    private static final double DEPLOYED = 0.4;
    private static final double MAX_CLIMB = 0.11;
    // private static final double INCREASE_POWER_THRESHOLD = .135;
    
    private final ClimberIO io;
    private final ClimberInputs inputs;

    public Climber() {
        inputs = new ClimberInputs();
        if (Robot.isReal()) {
            io = new ClimberIOTalonFX();
        } else {
            io = new ClimberIOSim();
        }
    }

    @Override
    public void periodic() {
        io.update(inputs);
    }

    public Command deploy() {
        return runSpeed(0.07).until(this::isDeployed); // was 0.06, 0.05
    }

    final Command climbDoneRumble = new RumbleCommand(1).withTimeout(2);

    public Command climb() {
        return runEnd(
            () -> {
                // io.setSpeed(shouldIncreaseClimbPower() ? 1 : 0.3);
                io.setSpeed(0.3);
            },
            () -> io.setSpeed(0)
        ).until(this::shouldStopClimbing)
        .andThen(new ScheduleCommand(climbDoneRumble));
        //.andThen(Commands.runOnce(() -> climbDoneRumble.schedule()));
    }

    public Command runSpeed(double speed) {
        return runEnd(
            () -> io.setSpeed(speed),
            () -> io.setSpeed(0)
        );
    }

    public boolean isDeployed() {
        return inputs.jointPosition > DEPLOYED;
    }

    public boolean shouldStopClimbing() {
        return inputs.jointPosition < MAX_CLIMB;
    }

    // public boolean shouldIncreaseClimbPower() {
    //     return inputs.jointPosition < INCREASE_POWER_THRESHOLD;
    // }
}
