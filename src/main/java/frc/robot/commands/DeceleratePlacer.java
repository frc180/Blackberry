package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CoralPlacerSubsystem;

public class DeceleratePlacer extends Command {
    private final CoralPlacerSubsystem placerSubsystem;
    private final double initialSpeed;
    private final Timer timer = new Timer();

    public DeceleratePlacer(CoralPlacerSubsystem placerSubsystem, double initialSpeed) {
        this.placerSubsystem = placerSubsystem;
        this.initialSpeed = initialSpeed;
        addRequirements(placerSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double elapsed = timer.get();
        double decelerationTime = Constants.Commands.CORAL_PLACEMENT_DECELERATION_SECONDS;
        double fractionRemaining = Math.max(0, 1 - (elapsed / decelerationTime));
        double currentSpeed = initialSpeed * fractionRemaining;
        placerSubsystem.runPlacer(currentSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        placerSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(Constants.Commands.CORAL_PLACEMENT_DECELERATION_SECONDS);
    }
}