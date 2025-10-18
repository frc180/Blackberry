package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralPlacerSubsystem;

public class CoralPlacementCycle extends Command {

    private final CoralPlacerSubsystem placerSubsystem;
    private final double speed;

    public CoralPlacementCycle(CoralPlacerSubsystem placerSubsystem, double speed) {
        this.placerSubsystem = placerSubsystem;
        this.speed = speed;
        addRequirements(placerSubsystem);
    }

    @Override
    public void execute() {
        placerSubsystem.runPlacer(speed);
    }

    @Override
    public void end(boolean interrupted) {
        // if (interrupted) {
        //     CommandScheduler.getInstance().schedule(new DeceleratePlacer(placerSubsystem, speed));
        // } else {
        //     placerSubsystem.stop();
        // }
        placerSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}