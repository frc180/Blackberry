package frc.robot.subsystems.climber;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberIO.ClimberInputs;

@Logged
public class Climber extends SubsystemBase {
    
    private final ClimberIO io;
    private final ClimberInputs inputs;

    public Climber() {
        inputs = new ClimberInputs();
        io = new ClimberIOSim();
    }

    /**
     * "driver activates deploy, winch retracts at low power until until it sees motion
     * on joint encoder indicating mechanism has been released."
     */
    public Command deploy() {
        return runSpeed(0.1).until(this::isDeployed);
    }

    /**
     * "winch retracts while held until driver releases or until max retract angle is achieved. ""
     */
    public Command climb() {
        return runSpeed(-0.5).until(this::shouldStopClimbing);
    }

    public Command runSpeed(double speed) {
        return runEnd(
            () -> io.setSpeed(speed),
            () -> io.setSpeed(0)
        );
    }

    public boolean isDeployed() {
        return inputs.jointPosition > 0.5;
    }

    public boolean shouldStopClimbing() {
        return inputs.jointPosition < 0.2;
    }
}
