package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.CoralPlacerSubsystem;

public class CoralPlacementCycle extends Command {
    private enum MotorState {
        FULL_SPEED,
        DECELERATING
    }

    private final CoralPlacerSubsystem placerSubsystem;
    private final double speed;
    private final double duration;
    private double startTime;
    private MotorState motorState;
    private final Timer decelerationTimer = new Timer();

    public CoralPlacementCycle(CoralPlacerSubsystem placerSubsystem, double speed, double duration) {
        this.placerSubsystem = placerSubsystem;
        this.speed = speed;
        this.duration = duration;
        addRequirements(placerSubsystem);
    }

    public CoralPlacementCycle(CoralPlacerSubsystem placerSubsystem, double speed) {
        this(placerSubsystem, speed, -1);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        motorState = MotorState.FULL_SPEED;
        decelerationTimer.stop();
        decelerationTimer.reset();
    }

    @Override
    public void execute() {
        if (motorState == MotorState.FULL_SPEED) {
            placerSubsystem.runPlacer(speed);
        } else { 
            double elapsed = decelerationTimer.get();
            double decelerationTime = Constants.Commands.CORAL_PLACEMENT_DECELERATION_SECONDS;
            double fractionRemaining = Math.max(0, 1 - (elapsed / decelerationTime));
            double currentSpeed = speed * fractionRemaining; 
            placerSubsystem.runPlacer(currentSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted && duration < 0) { 
            CommandScheduler.getInstance().schedule(new DeceleratePlacer(placerSubsystem, speed));
        } else {
            placerSubsystem.stop(); 
        }
    }

    @Override
    public boolean isFinished() {
        if (duration < 0) {
            return false; 
        }

        if (motorState == MotorState.FULL_SPEED) {
            if (Timer.getFPGATimestamp() - startTime >= duration) {
                motorState = MotorState.DECELERATING;
                decelerationTimer.reset();
                decelerationTimer.start();
            }
            return false;
        } else { 
            return decelerationTimer.hasElapsed(Constants.Commands.CORAL_PLACEMENT_DECELERATION_SECONDS);
        }
    }
}